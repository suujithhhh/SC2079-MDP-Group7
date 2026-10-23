"""Train v3; choose a checkpoint using fixed-confidence circle and replay checks."""
import gc,json,shutil
from pathlib import Path
import cv2,torch
from ultralytics import YOLO
from train_circle import evaluate,gates,summarize,ROOT

DATA=ROOT/'circle_data_v3';RUN=ROOT/'runs/yolo26s_seg_mdp_circle_v3';REVIEW=ROOT/'runs/yolo26s_seg_mdp_circle_v3_review'
BASE=ROOT/'runs/yolo26s_seg_mdp_circle_v2/weights/best.pt'

def paths(name):return [Path(s) for s in (DATA/f'{name}.txt').read_text().splitlines()]
def brief(report):
    rows=report['records'];native=[r for r in rows if '/real_eval/' in r['path'] and '_r160.' not in r['path']]
    reduced=[r for r in rows if '/real_eval/' in r['path'] and '_r160.' in r['path']]
    synthetic=[r for r in rows if '/real_eval/' not in r['path']]
    stats=summarize(synthetic)
    return {'native_real_correct':sum(r['returned']=='40' for r in native),'native_real_total':len(native),
            'reduced_real_correct':sum(r['returned']=='40' for r in reduced),'reduced_real_total':len(reduced),
            'native_predictions':[{k:r[k] for k in ['path','returned','confidence']} for r in native],
            'target':stats}
def rank(info):
    return (info['native_real_correct'],info['reduced_real_correct'],info['target']['40']['correct_return'],
            sum(v['correct_return'] for v in info['target'].values()))
def subset(report,n):return dict(report,records=report['records'][:n])

def main():
    cv2.setNumThreads(1);torch.set_num_threads(4)
    assert json.loads((DATA/'manifest.json').read_text())['complete']
    REVIEW.mkdir(exist_ok=True)
    legacy=[Path(p) for p in (ROOT/'circle_data_v2/selection_val.txt').read_text().splitlines()]
    target=paths('target_val')+paths('real_eval')
    baseline=evaluate(BASE,legacy,REVIEW/'baseline_legacy.json','0',32)
    initial=evaluate(BASE,target,REVIEW/'baseline_target.json','0',32)
    (REVIEW/'baseline_summary.json').write_text(json.dumps(brief(initial),indent=2)+'\n')
    best_rank=[(-1,)];epoch_scores=[]
    def monitor(trainer):
        epoch=trainer.epoch+1
        report=evaluate(trainer.last,target,REVIEW/f'epoch_{epoch:02d}_target.json','0',32)
        info=brief(report);info['epoch']=epoch;epoch_scores.append(info)
        if rank(info)>best_rank[0]:
            best_rank[0]=rank(info);shutil.copy2(trainer.last,RUN/'weights/best_target.pt')
        (REVIEW/'epoch_scores.json').write_text(json.dumps(epoch_scores,indent=2)+'\n')
        print('TARGET_PROGRESS',json.dumps({'epoch':epoch,'native_real':info['native_real_correct'],'reduced_real':info['reduced_real_correct'],'target_circle':info['target']['40']['correct_return']}),flush=True)
        gc.collect()
    model=YOLO(str(BASE));model.add_callback('on_model_save',monitor)
    model.train(data=str(DATA/'data.yaml'),epochs=10,patience=0,batch=32,imgsz=640,workers=2,device='0',
                project=str(RUN.parent),name=RUN.name,exist_ok=False,optimizer='AdamW',lr0=.0002,lrf=.15,
                warmup_epochs=1,warmup_bias_lr=0,seed=0,deterministic=True,amp=True,
                fliplr=0,flipud=0,copy_paste=0,degrees=5,translate=.1,scale=.2,hsv_h=.015,hsv_s=.3,hsv_v=.3,
                mosaic=.1,close_mosaic=3,mixup=0,save=True,save_period=1,plots=True)
    del model;gc.collect();torch.cuda.empty_cache()
    # Inspect the best target epochs and the ordinary fitness checkpoints.
    ranked=sorted(epoch_scores,key=rank,reverse=True)
    candidates=[('best_target',RUN/'weights/best_target.pt'),('best',RUN/'weights/best.pt'),('last',RUN/'weights/last.pt')]
    for info in ranked[:3]:
        p=RUN/'weights'/f'epoch{info["epoch"]-1}.pt'
        if p.exists():candidates.append((f'epoch_{info["epoch"]:02d}',p))
    results={}
    for name,checkpoint in candidates:
        replay=evaluate(checkpoint,legacy,REVIEW/f'{name}_legacy.json','0',32)
        target_report=evaluate(checkpoint,target,REVIEW/f'{name}_target.json','0',32)
        check=gates(baseline,replay);info=brief(target_report)
        # Real input improvements must not come solely from predicting circle on
        # everything: also enforce controls on the new small/dense-pattern set.
        before=brief(initial)['target'];after=info['target']
        checks={'real_native_improved':info['native_real_correct']>brief(initial)['native_real_correct'],
                'new_circle_recall_not_worse':after['40']['correct_return']>=before['40']['correct_return'],
                'new_six_correct_not_worse':after['16']['correct_return']>=before['16']['correct_return'],
                'new_false_circles_not_worse':sum(r['returned']=='40' and r['expected']!='40' for r in target_report['records'])<=sum(r['returned']=='40' and r['expected']!='40' for r in initial['records'])}
        results[name]={'checkpoint':str(checkpoint),'legacy_check':check,'target_checks':checks,'summary':info,'passed':check['passed'] and all(checks.values())}
    eligible=[n for n,r in results.items() if r['passed']]
    selected=max(eligible,key=lambda n:rank(results[n]['summary'])) if eligible else None
    if selected:shutil.copy2(results[selected]['checkpoint'],RUN/'weights/candidate.pt')
    report={'baseline':brief(initial),'candidates':results,'selected':selected,'run':str(RUN),
            'status':'requires_ncnn_validation' if selected else 'no_candidate_passed','production_model_replaced':False,
            'real_screenshots_used_for_checkpoint_selection':True,'original_test_used_for_selection':False}
    (REVIEW/'comparison.json').write_text(json.dumps(report,indent=2)+'\n')
    print('TRAINING_COMPLETE',selected,flush=True)
if __name__=='__main__':main()
