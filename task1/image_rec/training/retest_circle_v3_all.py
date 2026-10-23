"""Fresh all-class v3 NCNN retest, with matched v2 baseline and per-ID reports."""
import csv,hashlib,json,time
from collections import Counter
from pathlib import Path
import cv2,ncnn,numpy as np,torch,yaml
from train_circle import ROOT,evaluate
from circle_data import NAMES,label_for,labels
from inference import SYMBOL

OUT=ROOT/'runs/circle_v3_all_targets_retest'

def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def metric(rows):
    positives=[r for r in rows if r['expected'] is not None];empty=[r for r in rows if r['expected'] is None]
    correct=sum(r['returned']==r['expected'] for r in rows)
    per={}
    for ident in NAMES:
        own=[r for r in positives if r['expected']==ident];tp=sum(r['returned']==ident for r in own)
        predicted=sum(r['returned']==ident for r in rows);p=tp/predicted if predicted else 0.;rec=tp/len(own) if own else 0.
        per[ident]={'symbol':SYMBOL[ident],'images':len(own),'correct':tp,'missed':sum(r['returned'] is None for r in own),
                    'wrong_id':sum(r['returned'] not in [None,ident] for r in own),'localized_correct':sum(r['matched']==ident for r in own),
                    'precision':p,'recall':rec,'f1':2*p*rec/(p+rec) if p+rec else 0.,'predicted':predicted}
    return {'images':len(rows),'correct_returns':correct,'overall_accuracy':correct/len(rows),
            'positive_images':len(positives),'positive_correct':sum(r['returned']==r['expected'] for r in positives),
            'misses':sum(r['returned'] is None for r in positives),
            'wrong_ids':sum(r['returned'] is not None and r['returned']!=r['expected'] for r in positives),
            'empty_images':len(empty),'empty_false_detections':sum(r['returned'] is not None for r in empty),
            'macro_f1':float(np.mean([v['f1'] for v in per.values() if v['images']])),
            'per_class':per}

def write_tables(reports):
    summary={}
    for condition,models in reports.items():
        summary[condition]={tag:metric(r['records']) for tag,r in models.items()}
        old,new=summary[condition]['v2'],summary[condition]['v3']
        comparison=[]
        for ident in NAMES:
            a,b=old['per_class'][ident],new['per_class'][ident]
            comparison.append({'image_id':ident,'symbol':b['symbol'],'images':b['images'],
                'v2_correct':a['correct'],'v3_correct':b['correct'],'correct_change':b['correct']-a['correct'],
                'v3_missed':b['missed'],'v3_wrong_id':b['wrong_id'],'v3_precision':b['precision'],'v3_recall':b['recall'],'v3_f1':b['f1'],
                'v2_localized_correct':a['localized_correct'],'v3_localized_correct':b['localized_correct']})
        with (OUT/f'{condition}_per_class.csv').open('w') as f:
            writer=csv.DictWriter(f,fieldnames=list(comparison[0]));writer.writeheader();writer.writerows(comparison)
        ordered=NAMES+['none'];matrix=np.zeros((32,32),int)
        for r in models['v3']['records']:matrix[ordered.index(r['expected'] or 'none'),ordered.index(r['returned'] or 'none')]+=1
        with (OUT/f'{condition}_confusion.csv').open('w') as f:
            w=csv.writer(f);w.writerow(['expected/returned']+ordered)
            for ident,row in zip(ordered,matrix):w.writerow([ident]+row.tolist())
        errors=[dict(r,source_condition=condition) for r in models['v3']['records'] if r['returned']!=r['expected']]
        (OUT/f'{condition}_errors.json').write_text(json.dumps(errors,indent=2)+'\n')
    return summary

def main():
    OUT.mkdir(exist_ok=True);cv2.setNumThreads(1);torch.set_num_threads(4)
    old_net=ncnn.Net
    def limited(*a,**kw):
        net=old_net(*a,**kw);net.opt.num_threads=4;return net
    ncnn.Net=limited
    models={tag:ROOT/'handoff'/f'mdp_circle_{tag}_ncnn_model' for tag in ['v2','v3']}
    hashes={tag:digest(p/'model.ncnn.bin') for tag,p in models.items()}
    for tag,p in models.items():assert hashes[tag]==json.loads((p/'manifest.json').read_text())['sha256']['model.ncnn.bin']
    native_val=sorted((ROOT/'valid/images').glob('*.jpg'));native_test=sorted((ROOT/'test/images').glob('*.jpg'))
    # Locate exact original160 validation paths, excluding synthetic replay.
    baseline_val=json.loads((ROOT/'runs/yolo26s_seg_mdp_circle_v2_review/candidate_ncnn.json').read_text())
    val_rows=[r for r in baseline_val['records'] if '/lowres_data_160/val/' in r['path']]
    assert len(native_val)==len(val_rows)==778 and len(native_test)==335
    (OUT/'inputs/test_160/images').mkdir(parents=True,exist_ok=True);(OUT/'inputs/test_160/labels').mkdir(exist_ok=True)
    reduced=[]
    for p in native_test:
        im=cv2.imread(str(p));h,w=im.shape[:2];scale=160/max(h,w);target=OUT/'inputs/test_160/images'/p.name
        cv2.imwrite(str(target),cv2.resize(im,(round(w*scale),round(h*scale)),interpolation=cv2.INTER_AREA),[cv2.IMWRITE_JPEG_QUALITY,95])
        label_for(target).write_text(label_for(p).read_text());reduced.append(target)
    sets={'validation_native':native_val,'validation_160':[Path(r['path']) for r in val_rows],
          'test_native':native_test,'test_160':reduced}
    for name,ps in sets.items():
        for p in ps:assert len(labels(label_for(p)))<=1,p
        (OUT/f'{name}_images.txt').write_text('\n'.join(map(str,ps))+'\n')
    config={'path':str(ROOT),'train':str(ROOT/'train/images'),'val':str(ROOT/'valid/images'),'test':str(ROOT/'test/images'),'nc':31,'names':NAMES}
    (OUT/'metrics_data.yaml').write_text(yaml.safe_dump(config,sort_keys=False))
    reports={}
    baseline_test=json.loads((ROOT/'runs/yolo26s_seg_mdp_circle_v2_review/test_candidate_ncnn.json').read_text())
    # These two unchanged v2 baselines were already measured in the exact NCNN
    # runtime. Reuse them transparently; all v3 entries are freshly predicted.
    cached_v2={'validation_160':dict(baseline_val,records=val_rows),'test_native':baseline_test}
    provenance={};times={}
    for condition,ps in sets.items():
        reports[condition]={}
        for tag,model in models.items():
            key=f'{tag}_{condition}';dest=OUT/f'{key}.json'
            if tag=='v2' and condition in cached_v2:
                report=cached_v2[condition]
                assert [r['path'] for r in report['records']]==list(map(str,ps))
                provenance[key]='previous NCNN baseline, model hash verified unchanged'
            else:
                start=time.perf_counter();report=evaluate(model,ps,dest,'cpu',1);times[key]=time.perf_counter()-start
                provenance[key]='fresh NCNN evaluation'
            reports[condition][tag]=report
            summary=write_tables(reports if all(len(v)==2 for v in reports.values()) else {k:v for k,v in reports.items() if len(v)==2})
            status={'completed':list(provenance),'summary':summary};(OUT/'progress.json').write_text(json.dumps(status,indent=2)+'\n')
            print('RETEST_COMPLETE',key,metric(report['records'])['correct_returns'],'/',len(ps),flush=True)
    summary=write_tables(reports)
    result={'models':{tag:str(p) for tag,p in models.items()},'model_sha256':hashes,'network_imgsz':640,'confidence':.35,
            'selection':'class-agnostic IoU .5 dedup, largest box, no C/L/R signal','classes':NAMES,'summary':summary,
            'provenance':provenance,'wall_seconds':times,
            'limitations':['Native and160 sets are paired variants, not independent images.',
            'Validation was used during training/selection. Legacy data has known duplicates.',
            'Test split was not used for the circle-v3 checkpoint selection.',
            'Host CPU NCNN evaluation does not measure Pi latency.']}
    (OUT/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print('ALL_TARGET_RETEST_DONE',flush=True)
if __name__=='__main__':main()
