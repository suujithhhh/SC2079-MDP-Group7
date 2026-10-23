"""Evaluate deployment predictions, size-band recall, direction errors and negatives."""
from pathlib import Path
import argparse,json,time,collections
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
NAMES=['bullseye','left','right']; LEGACY={'marker':0,'39':1,'38':2}

def iou(a,b):
    a=np.asarray(a);b=np.asarray(b);d=np.maximum(0,np.minimum(a[2:],b[2:])-np.maximum(a[:2],b[:2]));inter=float(d.prod())
    return inter/max(1e-9,float((a[2:]-a[:2]).prod()+(b[2:]-b[:2]).prod()-inter))

def summarize(records,conf=.45):
    groups=collections.defaultdict(lambda:dict(objects=0,correct=0,missed=0,wrong_class=0,wrong_direction=0,false_positives=0,images=0,negative_images=0,negative_images_with_fp=0))
    errors=[]
    for r in records:
        gt=r['objects'];pred=[p for p in r['predictions'] if p['confidence']>=conf];used=set();matches=[]
        # Class-independent matching makes an opposite-arrow prediction an error, not a miss hidden by filtering.
        candidates=sorted([(iou(g['box'],p['box']),gi,pi) for gi,g in enumerate(gt) for pi,p in enumerate(pred)],reverse=True)
        matched={};gs=set()
        for overlap,gi,pi in candidates:
            if overlap<.3:break
            if gi not in gs and pi not in used:matched[gi]=pi;gs.add(gi);used.add(pi)
        kinds=['all',r['kind']]
        for kind in kinds:
            s=groups[kind];s['images']+=1;s['false_positives']+=len(pred)-len(used)
            if not gt:s['negative_images']+=1;s['negative_images_with_fp']+=bool(pred)
        bad=False
        for gi,g in enumerate(gt):
            p=pred[matched[gi]] if gi in matched else None
            correct=p is not None and p['class']==g['class'];wrong=p is not None and not correct
            direction=wrong and g['class'] in [1,2] and p['class'] in [1,2]
            size=max(g['box'][2]-g['box'][0],g['box'][3]-g['box'][1])
            band='<20' if size<20 else '20-32' if size<32 else '32-48' if size<48 else '48-80' if size<80 else '80+'
            for key in kinds+[f"{r['kind']}/{NAMES[g['class']]}",f"{r['kind']}/size_{band}"]:
                s=groups[key];s['objects']+=1;s['correct']+=int(correct);s['missed']+=int(p is None);s['wrong_class']+=int(wrong);s['wrong_direction']+=int(direction)
            if not correct:bad=True
        if bad or len(pred)>len(used):errors.append({'path':r['path'],'objects':gt,'predictions':pred,'kind':r['kind']})
    for v in groups.values():v['recall']=v['correct']/v['objects'] if v['objects'] else None
    return {'confidence':conf,'matching_iou':.3,'reason_for_iou':'Class/turn recognition score; legacy arrow boxes cover only arrowheads. Standard new-model box AP is reported separately.','groups':dict(groups),'errors':errors}

def evaluate(model_path,split='val',imgsz=640,device='0',out=None,legacy=False):
    import cv2,torch
    from ultralytics import YOLO
    cv2.setNumThreads(1);torch.set_num_threads(4)
    if Path(model_path).is_dir():
        import ncnn
        original=ncnn.Net
        def limited(*a,**kw):
            net=original(*a,**kw);net.opt.num_threads=4;return net
        ncnn.Net=limited
    model=YOLO(str(model_path),task='segment' if legacy else 'detect')
    if Path(model_path).is_dir():
        try:
            model.predict(np.zeros((imgsz,imgsz,3),dtype=np.uint8),imgsz=imgsz,device='cpu',conf=.05,verbose=False,rect=False,end2end=False)
        finally:
            ncnn.Net=original
    records=[r for r in json.loads((ROOT/'data/manifest.json').read_text())['records'] if r['split']==split]
    batch=1 if device=='cpu' else 32;result_rows=[];start=time.perf_counter()
    for off in range(0,len(records),batch):
        rr=records[off:off+batch]
        results=model.predict([r['path'] for r in rr],imgsz=imgsz,device=device,conf=.05,iou=.5,max_det=20,batch=batch,verbose=False,rect=False,end2end=False)
        for row,result in zip(rr,results):
            preds=[]
            for box in result.boxes:
                cls=int(box.cls.item());name=result.names[cls]
                if legacy:
                    if name not in LEGACY:continue
                    cls=LEGACY[name]
                preds.append({'class':cls,'confidence':float(box.conf.item()),'box':box.xyxy[0].cpu().tolist()})
            result_rows.append({**row,'predictions':preds})
        if off%128==0:print('EVAL_PROGRESS',Path(model_path).name,split,imgsz,off,len(records),flush=True)
    report={'model':str(model_path),'split':split,'imgsz':imgsz,'size_band_reference':'Maximum full-glyph side in the 640x640 source frame; multiply by imgsz/640 for network-input pixels.','baseline_filter':'Only marker/39/38 retained; not the old application largest-box policy.' if legacy else None,'device':device,'end2end':False,'seconds':time.perf_counter()-start,'records':result_rows,'summary':summarize(result_rows)}
    if out:Path(out).write_text(json.dumps(report,indent=2)+'\n')
    return report

def main():
    p=argparse.ArgumentParser();p.add_argument('model');p.add_argument('--split',default='val');p.add_argument('--imgsz',type=int,default=640);p.add_argument('--device',default='0');p.add_argument('--out',required=True);p.add_argument('--legacy',action='store_true');a=p.parse_args()
    r=evaluate(a.model,a.split,a.imgsz,a.device,a.out,a.legacy);print(json.dumps(r['summary']['groups'],indent=2),flush=True)
if __name__=='__main__':main()
