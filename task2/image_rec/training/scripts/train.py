"""Reproducible Task 2 nano detector training, with sampled deployment-head validation."""
from pathlib import Path
import json, shutil, os
ROOT=Path(__file__).resolve().parents[1]
os.environ.setdefault('YOLO_CONFIG_DIR',str(ROOT/'config'))
os.environ.setdefault('YOLO_AUTOINSTALL','false')
import cv2,torch
from ultralytics import YOLO,settings
from evaluate import evaluate

def main():
    if (ROOT/'runs/yolo26n_task2_v1').exists():raise FileExistsError('Run already exists; use a new explicit run name and update export path.')
    cv2.setNumThreads(1);torch.set_num_threads(4)
    settings.update({'runs_dir':str(ROOT/'runs'),'datasets_dir':str(ROOT/'data'),'weights_dir':str(ROOT/'weights'),'sync':False})
    base=ROOT/'weights/yolo26n.pt'
    if not base.exists():shutil.copy2(ROOT.parent/'weights/yolo26n.pt',base)
    manifest=json.loads((ROOT/'data/manifest.json').read_text());assert manifest['group_overlap']==0
    monitor_dir=ROOT/'reports/checkpoints';monitor_dir.mkdir(parents=True,exist_ok=True)
    best_score=[-1e9];monitor_scores=[]
    def monitor(trainer):
        epoch=trainer.epoch+1
        if epoch%5 and epoch!=trainer.epochs:return
        report=evaluate(trainer.last,'val',640,'0',monitor_dir/f'epoch_{epoch:02d}.json')
        s=report['summary']['groups']['all']
        score=s['correct']-10*s['wrong_class']-20*s['wrong_direction']-5*s['false_positives']
        monitor_scores.append({'epoch':epoch,'score':score,**s})
        if score>best_score[0]:
            best_score[0]=score;shutil.copy2(trainer.last,trainer.last.parent/'best_deployment.pt')
        (monitor_dir/'selection.json').write_text(json.dumps({'criterion':'correct - 10*wrong_class - 20*wrong_direction - 5*false_positives, validation only, conf0.45, IoU0.30','scores':monitor_scores},indent=2)+'\n')
        print('DEPLOYMENT_VALIDATION',epoch,json.dumps(s),flush=True)
        torch.cuda.empty_cache()
    model=YOLO(str(base));model.add_callback('on_model_save',monitor)
    model.train(data=str(ROOT/'data/data.yaml'),project=str(ROOT/'runs'),name='yolo26n_task2_v1',
       epochs=50,patience=12,imgsz=640,batch=64,device=0,workers=4,optimizer='AdamW',lr0=.001,lrf=.05,
       cos_lr=True,warmup_epochs=3,warmup_bias_lr=0,seed=2602,deterministic=True,
       fliplr=0,flipud=0,degrees=4,translate=.08,scale=.3,mosaic=.3,close_mosaic=10,
       mixup=0,copy_paste=0,hsv_h=0,hsv_s=.15,hsv_v=.25,amp=True,cache=False,
       save=True,save_period=10,plots=True,end2end=False)
    print('TRAINING_FINISHED',flush=True)
if __name__=='__main__':main()
