"""Choose by validation, export fixed NCNN resolutions, then evaluate untouched test."""
from pathlib import Path
import json,shutil,hashlib,time,zipfile,platform
import numpy as np
import cv2,torch
from ultralytics import YOLO
from ultralytics.utils.torch_utils import strip_optimizer
from evaluate import evaluate,summarize
ROOT=Path(__file__).resolve().parents[1]; RUN=ROOT/'runs/yolo26n_task2_v1';REPORT=ROOT/'reports'

def score(r,conf=.45):
    s=summarize(r['records'],conf)['groups']['all']
    return (-s['wrong_direction'],-s['wrong_class'],-s['false_positives'],s['correct'])
def benchmark(folder,size,paths,conf,task="detect"):
    import ncnn
    torch.set_num_threads(4);cv2.setNumThreads(1)
    orig=ncnn.Net
    def limited(*a,**kw):
        net=orig(*a,**kw);net.opt.num_threads=4;return net
    ncnn.Net=limited
    model=YOLO(str(folder),task=task);frames=[cv2.imread(p) for p in paths[:12]]
    try:
        for i in range(5):model.predict(frames[i%len(frames)],imgsz=size,conf=conf,end2end=False,rect=False,verbose=False,device='cpu')
    finally:ncnn.Net=orig
    times=[]
    for i in range(36):
        t=time.perf_counter();model.predict(frames[i%len(frames)],imgsz=size,conf=conf,end2end=False,rect=False,verbose=False,device='cpu');times.append((time.perf_counter()-t)*1000)
    return {'median_ms':float(np.median(times)),'p90_ms':float(np.percentile(times,90)),'mean_ms':float(np.mean(times)),'frames':len(times),'threads':4,'platform':platform.platform(),'processor':json.loads((REPORT/'environment.json').read_text())['cpu'],'confidence':conf,'includes':'preprocessing + NCNN + NMS + result construction','excludes':'camera capture, file decode, transfer, navigation','is_pi_measurement':False}

def main():
    torch.set_num_threads(4);cv2.setNumThreads(1)
    candidates={}
    names=['best_deployment','best','last']+sorted(p.stem for p in (RUN/'weights').glob('epoch*.pt'))
    for name in names:
        path=RUN/'weights'/f'{name}.pt'
        if path.exists():
            out=REPORT/f'{name}_val_pt640.json'
            r=evaluate(path,'val',640,'0',out)
            points={str(c):{'score':score(r,c),'summary':summarize(r['records'],c)['groups']} for c in [.45,.55,.65,.75,.85,.9,.95]}
            conf=max(points,key=lambda c:(points[c]['score'],-float(c)))
            candidates[name]={'score':points[conf]['score'],'confidence':float(conf),'operating_points':points,'summary':points[conf]['summary']}
    selected=max(candidates,key=lambda n:(candidates[n]['score'],-candidates[n]['confidence']))
    conf=candidates[selected]['confidence']
    final=ROOT/'weights/best.pt';strip_optimizer(RUN/'weights'/f'{selected}.pt',s=str(final))
    selection={'criterion':'lexicographic: fewest opposite-arrow errors, fewest wrong classes, fewest extra detections, then most correct; confidence sweep validation only','selected':selected,'candidates':candidates,'confidence':conf,'checkpoint':str(final),'test_used_for_selection':False}
    (REPORT/'checkpoint_selection.json').write_text(json.dumps(selection,indent=2)+'\n')
    exports={}
    for size in [640,512]:
        staging=ROOT/'weights'/f'task2_{size}.pt';shutil.copy2(final,staging)
        folder=Path(YOLO(str(staging)).export(format='ncnn',imgsz=size,batch=1,half=False,device='cpu'))
        out=REPORT/f'task2_val_ncnn{size}.json';r=evaluate(folder,'val',size,'cpu',out)
        r['summary']=summarize(r['records'],conf);out.write_text(json.dumps(r,indent=2)+'\n')
        exports[size]={'folder':str(folder),'val_score':score(r,conf),'validation':r['summary']['groups']}
    recommended=max(exports,key=lambda n:(exports[n]['val_score'],n))
    test_reports={}
    for size,info in exports.items():
        out=REPORT/f'task2_test_ncnn{size}.json';r=evaluate(info['folder'],'test',size,'cpu',out)
        r['summary']=summarize(r['records'],conf);out.write_text(json.dumps(r,indent=2)+'\n')
        test_reports[str(size)]=r['summary']['groups']
    baseline=ROOT.parent/'handoff/mdp_circle_v3_ncnn_model'
    baseline_report=evaluate(baseline,'test',640,'cpu',REPORT/'baseline_v3_test_ncnn640.json',legacy=True)
    baseline_report['summary']=summarize(baseline_report['records'],.35)
    (REPORT/'baseline_v3_test_ncnn640.json').write_text(json.dumps(baseline_report,indent=2)+'\n')
    ap={}
    for split in ['val','test']:
        for size in [640,512]:
            metrics=YOLO(str(final)).val(data=str(ROOT/'data/data.yaml'),split=split,imgsz=size,batch=32,device=0,workers=2,end2end=False,rect=False,plots=True,project=str(REPORT/'ap'),name=f'{split}_{size}',exist_ok=True)
            ap[f'{split}_{size}']={'box_map50':float(metrics.box.map50),'box_map50_95':float(metrics.box.map),'per_class_ap50':metrics.box.ap50.tolist(),'per_class_ap50_95':metrics.box.ap.tolist()}
    paths=[r['path'] for r in json.loads((ROOT/'data/manifest.json').read_text())['records'] if r['split']=='test' and r['kind']=='real']
    speeds={str(s):benchmark(i['folder'],s,paths,conf) for s,i in exports.items()}
    speeds['baseline_v3_640']=benchmark(baseline,640,paths,.35,task='segment')
    result={'selected_checkpoint':selection,'recommended_imgsz':recommended,'confidence':conf,'validation':exports,'test':test_reports,'baseline_confidence':.35,'baseline_test':baseline_report['summary']['groups'],'standard_ap':ap,'host_benchmarks':speeds,'limitations':['No live Task2 Pi camera frames were provided.','Synthetic pixel-size tests are not physical detection-distance measurements.','Source groups are disjoint; nearby groups can still share room/capture session.','Legacy baseline had previously trained on some regrouped holdout sources; it is a reference, not an independently held-out retraining baseline.','NCNN host timing is not Pi timing.','Three-frame confirmation and navigation behavior require integration and live testing.']}
    (REPORT/'results.json').write_text(json.dumps(result,indent=2)+'\n')
    for size,info in exports.items():
        package=ROOT/'handoff'/f'task2_yolo26n_{size}_ncnn_model';package.mkdir(exist_ok=True)
        for name in ['model.ncnn.param','model.ncnn.bin','metadata.yaml']:shutil.copy2(Path(info['folder'])/name,package/name)
        shutil.copy2(ROOT/'scripts/inference.py',package/'inference.py')
        (package/'operating_point.json').write_text(json.dumps({'confidence':conf,'imgsz':size,'selected_on':'validation only','confirm_frames':3,'min_arrow_input_side':12.0},indent=2)+'\n')
        (package/'requirements.txt').write_text('ultralytics==8.4.128\nncnn==1.0.20260526\nPillow\nPyYAML\n# Use the existing platform-compatible torch/torchvision and OpenCV installation.\n')
        (package/'README.txt').write_text(f'Task2 YOLO26n: bullseye, left, right. Fixed input {size}. Recommended confidence{conf} (selected on validation only).\nRun: python inference.py IMAGE.jpg --model . --evidence-dir evidence\nLocal classes0/1/2 map to legacy IDs marker/39/38. Do not use local class index as robot image ID.\nRAW inputs are preserved separately from annotated evidence and a tiled contact sheet.\nSingle image is an observation, never a confirmed turn. Use Detector and ArrowConfirmation\nwith a live-selected obstacle ROI, consecutive camera frames, and reset between obstacles.\nThis package does not control or stop the robot.\nHost-only validation; measure on your Pi and live white-background cards before a competition.\n')
        shutil.copy2(REPORT/'results.json',package/'validation.json')
        status='Recommended for live Pi testing; not validated on the robot.' if size==recommended else 'Comparison export only: not recommended; 512 had an opposite-arrow validation error.'
        with (package/'README.txt').open('a') as f:
            f.write('\n'+status+'\nEarly candidates: add --observation-conf 0.35; confirmation remains at the selected threshold.\nUse original camera frames at least640pixels wide. Arrow confirmation requires three frames and at least12pixels across the smaller side in both source and network images.\n')
        hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in package.iterdir() if p.is_file() and p.name!='manifest.json'}
        (package/'manifest.json').write_text(json.dumps({'source_checkpoint_sha256':hashlib.sha256(final.read_bytes()).hexdigest(),'imgsz':size,'recommended':size==recommended,'candidate_only':True,'requires_live_pi_validation':True,'files':hashes},indent=2)+'\n')
        with zipfile.ZipFile(package.with_suffix('.zip'),'w',zipfile.ZIP_DEFLATED) as z:
            for p in package.iterdir():
                if p.is_file():z.write(p,arcname=f'{package.name}/{p.name}')
        with zipfile.ZipFile(package.with_suffix('.zip')) as z:assert z.testzip() is None
    print('TASK2_EXPORT_COMPLETE',recommended,flush=True)
if __name__=='__main__':main()
