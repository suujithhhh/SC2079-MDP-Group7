"""Standard box/mask AP on the original test images and their 160px versions."""
import csv,json,gc
from pathlib import Path
import cv2,torch,yaml
from ultralytics import YOLO
from train_circle import ROOT
from inference import SYMBOL
OUT=ROOT/'runs/circle_v3_all_targets_retest'

def main():
    cv2.setNumThreads(1);torch.set_num_threads(4)
    config=yaml.safe_load((OUT/'metrics_data.yaml').read_text())
    reduced=dict(config,test=str(OUT/'inputs/test_160/images'))
    (OUT/'metrics_data_160.yaml').write_text(yaml.safe_dump(reduced,sort_keys=False))
    result={};rows=[]
    for condition,data in [('test_native',OUT/'metrics_data.yaml'),('test_160',OUT/'metrics_data_160.yaml')]:
        result[condition]={}
        for tag in ['v2','v3']:
            weights=ROOT/f'runs/yolo26s_seg_mdp_circle_{tag}/weights/best.pt'
            model=YOLO(str(weights))
            metrics=model.val(data=str(data),split='test',imgsz=640,batch=32,device='0',workers=2,
                end2end=False,rect=False,conf=.001,iou=.7,plots=True,project=str(OUT),name=f'{tag}_{condition}_ap',verbose=False)
            per=[]
            for idx,cls in enumerate(metrics.box.ap_class_index):
                ident=model.names[int(cls)];row={'condition':condition,'model':tag,'image_id':ident,'symbol':SYMBOL[ident],
                    'box_ap50':float(metrics.box.ap50[idx]),'box_ap50_95':float(metrics.box.ap[idx]),
                    'mask_ap50':float(metrics.seg.ap50[idx]),'mask_ap50_95':float(metrics.seg.ap[idx])}
                per.append(row);rows.append(row)
            result[condition][tag]={'weights':str(weights),'metrics':{k:float(v) for k,v in metrics.results_dict.items()},'per_class':per}
            (OUT/'standard_metrics.json').write_text(json.dumps({'backend':'PyTorch CUDA, end2end=False matching NCNN exported head',
                'confidence_for_AP':.001,'imgsz':640,'iou_nms':.7,'notes':'AP integrates confidence thresholds; fixed0.35 application-ID metrics are reported separately.',
                'results':result},indent=2)+'\n')
            print('STANDARD_METRICS',condition,tag,json.dumps(result[condition][tag]['metrics']),flush=True)
            del model,metrics;gc.collect();torch.cuda.empty_cache()
    with (OUT/'standard_metrics_per_class.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
if __name__=='__main__':main()
