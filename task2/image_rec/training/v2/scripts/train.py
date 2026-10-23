"""Fine-tune YOLO26n's actual NCNN detection head for Task 2 arrows."""
from pathlib import Path
import os,sys,json,shutil
ROOT=Path(__file__).resolve().parents[1]
os.environ.setdefault('YOLO_CONFIG_DIR',str(ROOT/'config'))
os.environ.setdefault('YOLO_AUTOINSTALL','false')
sys.path.insert(0,str(ROOT/'audit'))
import cv2,torch
from ultralytics import YOLO,settings
import ultralytics.data.build as data_build

class BoundedPrefetchLoader(data_build.InfiniteDataLoader):
    """Keep train + validation queues within the container 1GiB shared memory."""
    def __init__(self, *args, **kwargs):
        if kwargs.get("num_workers", 0)>0:kwargs["prefetch_factor"]=1
        super().__init__(*args, **kwargs)

data_build.InfiniteDataLoader=BoundedPrefetchLoader
from one2many_trainer import DeploymentHeadTrainer

class ArrowTrainer(DeploymentHeadTrainer):
    def get_dataloader(self, dataset_path, batch_size=16, rank=0, mode="train"):
        saved=self.args.workers
        if mode=="val":self.args.workers=0
        try:return super().get_dataloader(dataset_path,batch_size,rank,mode)
        finally:self.args.workers=saved
from evaluate import predict_records,summarize

def score(summary):
    s=summary['all'];arrows=sum(summary['by_class'][c]['correct'] for c in ['left','right'])
    return (s['wrong_direction'],s['wrong_class'],s['false_positives'],-arrows,-s['correct'])

def main():
    run=ROOT/'runs/yolo26n_task2_arrow_multi_v2'
    resume='--resume' in sys.argv
    if run.exists() and not resume:raise FileExistsError('Preserve existing run; use --resume or a new run directory.')
    cv2.setNumThreads(1);torch.set_num_threads(4)
    settings.update({'runs_dir':str(ROOT/'runs'),'datasets_dir':str(ROOT/'data'),'weights_dir':str(ROOT/'weights'),'sync':False})
    manifest=json.loads((ROOT/'data/manifest.json').read_text())
    assert not any(manifest['group_overlap'].values())
    val=[r for r in manifest['records'] if r['split']=='val']
    reports=ROOT/'reports/checkpoints_multi';reports.mkdir(parents=True,exist_ok=True)
    history=json.loads((reports/'selection.json').read_text())['history'] if resume and (reports/'selection.json').exists() else []
    best=[tuple(json.loads((reports/'best.json').read_text())['score']) if resume and (reports/'best.json').exists() else None]
    def monitor(trainer):
        epoch=trainer.epoch+1
        if epoch%3 and epoch!=trainer.epochs and not trainer.stop:return
        path=reports/f'epoch_{epoch:02d}.json'
        records=predict_records(trainer.last,val,640,'0',path)
        candidates=[summarize(records,c,.5) for c in [.45,.55,.65,.75,.85,.90,.95]]
        chosen=min(candidates,key=score);key=score(chosen)
        history.append({'epoch':epoch,'best_confidence':chosen['confidence'],'score':key,'summary':chosen,'grid':[{k:v for k,v in s.items() if k!='errors'} for s in candidates]})
        if best[0] is None or key<best[0]:
            best[0]=key;shutil.copy2(trainer.last,trainer.last.parent/'best_arrow.pt')
            (reports/'best.json').write_text(json.dumps({'epoch':epoch,'confidence':chosen['confidence'],'score':key,'summary':chosen},indent=2)+'\n')
        (reports/'selection.json').write_text(json.dumps({'criterion':'Validation only: minimize opposite directions, all wrong classes, extra detections; then maximize correct arrows and all targets. IoU0.5. Confidence grid .45,.55,.65,.75,.85,.90,.95.','history':history},indent=2)+'\n')
        print('ARROW_VALIDATION',epoch,chosen['confidence'],json.dumps(chosen['all']),flush=True)
        torch.cuda.empty_cache()
    model=YOLO(str(run/'weights/last.pt' if resume else ROOT/'runs/yolo26n_task2_arrow_v2/weights/last.pt'));model.add_callback('on_model_save',monitor)
    model.train(trainer=ArrowTrainer,data=str(ROOT/'data/data.yaml'),project=str(ROOT/'runs'),name=run.name,
        epochs=24,patience=10,imgsz=640,batch=64,device=0,workers=4,optimizer='AdamW',lr0=.0003,lrf=.05,
        cos_lr=True,warmup_epochs=2,warmup_bias_lr=0,seed=261013,deterministic=True,
        fliplr=0,flipud=0,degrees=3,translate=.04,scale=.20,mosaic=.10,close_mosaic=8,
        mixup=0,copy_paste=0,hsv_h=0,hsv_s=.08,hsv_v=.22,amp=True,cache="ram",
        save=True,save_period=5,plots=True,end2end=False,resume=resume)
    print('TRAINING_FINISHED',flush=True)
if __name__=='__main__':main()
