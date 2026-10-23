"""Validate and package the v3 NCNN candidate without replacing v2."""
import hashlib,json,shutil,zipfile
from pathlib import Path
import ncnn,torch
from ultralytics import YOLO
from train_circle import ROOT,evaluate,gates,summarize
from train_circle_v3 import brief
DATA=ROOT/'circle_data_v3';RUN=ROOT/'runs/yolo26s_seg_mdp_circle_v3';REVIEW=ROOT/'runs/yolo26s_seg_mdp_circle_v3_review'
def paths(name):return [Path(s) for s in (DATA/f'{name}.txt').read_text().splitlines()]
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    report=json.loads((REVIEW/'deploy_selection.json').read_text())
    if not report['selected']:raise SystemExit('No checkpoint passed deployment-head gates; review before export')
    checkpoint=RUN/'weights/candidate.pt'
    exported=Path(YOLO(str(checkpoint)).export(format='ncnn',imgsz=640,batch=1,half=False,device='cpu'))
    old_net=ncnn.Net
    def limited(*a,**kw):
        net=old_net(*a,**kw);net.opt.num_threads=4;return net
    ncnn.Net=limited;torch.set_num_threads(4)
    legacy=[Path(p) for p in (ROOT/'circle_data_v2/selection_val.txt').read_text().splitlines()]
    base=json.loads((ROOT/'runs/yolo26s_seg_mdp_circle_v2_review/candidate_ncnn.json').read_text())
    assert [r['path'] for r in base['records']]==[str(p) for p in legacy]
    target=paths('target_val')+paths('real_eval')
    old_target=evaluate(ROOT/'handoff/mdp_circle_v2_ncnn_model',target,REVIEW/'baseline_target_ncnn.json','cpu',1)
    new_target=evaluate(exported,target,REVIEW/'candidate_target_ncnn.json','cpu',1)
    new_legacy=evaluate(exported,legacy,REVIEW/'candidate_legacy_ncnn.json','cpu',1)
    checks=gates(base,new_legacy);old_info=brief(old_target);new_info=brief(new_target)
    native=[r for r in new_target['records'] if '/real_eval/' in r['path'] and '_r160.' not in r['path']]
    failures=[r for r in native if Path(r['path']).name.startswith(('capture_00_','capture_01_'))]
    extra={'heldout_native_improved':new_info['native_real_correct']>old_info['native_real_correct'],
           'original_two_failure_screenshots_correct':len(failures)==2 and all(r['returned']=='40' for r in failures),
           'new_circle_recall_not_worse':new_info['target']['40']['correct_return']>=old_info['target']['40']['correct_return'],
           'new_six_accuracy_not_worse':new_info['target']['16']['correct_return']>=old_info['target']['16']['correct_return'],
           'new_false_circles_not_worse':sum(r['returned']=='40' and r['expected']!='40' for r in new_target['records'])<=sum(r['returned']=='40' and r['expected']!='40' for r in old_target['records'])}
    passed=checks['passed'] and all(extra.values())
    result={'export':str(exported),'source_checkpoint':str(checkpoint),'selected_checkpoint':report['selected'],
            'legacy_check':checks,'target_checks':extra,'baseline':old_info,'candidate':new_info,
            'passed':passed,'production_model_replaced':False,'status':'candidate_for_pi_validation' if passed else 'requires_further_work',
            'limitations':['Five held-out source screenshots (two capture families); reduced copies are not independent samples.',
            'Screenshots include annotation pixels; actual Pi capture preprocessing and runtime have not been measured.',
            'Held-out screenshots were used for checkpoint selection; original test split was not.']}
    # Original test measures regressions after all checkpoint selection decisions.
    test=[Path(r['path']) for r in json.loads((ROOT/'runs/yolo26s_seg_mdp_circle_v2_review/test_candidate_ncnn.json').read_text())['records']]
    old_test=json.loads((ROOT/'runs/yolo26s_seg_mdp_circle_v2_review/test_candidate_ncnn.json').read_text())
    new_test=evaluate(exported,test,REVIEW/'test_candidate_ncnn.json','cpu',1)
    result['original_test']={'baseline':old_test['per_class'],'candidate':new_test['per_class'],'images':len(test),'used_for_selection':False}
    (REVIEW/'ncnn_comparison.json').write_text(json.dumps(result,indent=2)+'\n')
    if not passed:
        print('NCNN_CHECKS_FAILED',json.dumps(extra),flush=True);return
    package=ROOT/'handoff/mdp_circle_v3_ncnn_model'
    if package.exists():raise FileExistsError(package)
    package.mkdir()
    for name in ['model.ncnn.param','model.ncnn.bin','metadata.yaml']:shutil.copy2(exported/name,package/name)
    for name in ['inference.py','model_ncnn.py','requirements.txt']:shutil.copy2(ROOT/'handoff/mdp_circle_v2_ncnn_model'/name,package/name)
    shutil.copy2(REVIEW/'ncnn_comparison.json',package/'validation.json')
    shutil.copy2(DATA/'manifest.json',package/'training_data_manifest.json')
    shutil.copy2(ROOT/'handoff/mdp_circle_v2_ncnn_model/sample_circle_160.jpg',package/'sample_circle_160.jpg')
    shutil.copy2(ROOT/'handoff/mdp_circle_v2_ncnn_model/sample_six_160.jpg',package/'sample_six_160.jpg')
    (package/'README.txt').write_text('''Black-circle v3 NCNN candidate

Circle image ID40 (class29); six image ID16 (class5). All new circles are black.
Use network imgsz=640, batch1, FP32, confidence0.35. Keep the complete folder.
Run using the existing Pi Python environment:
  python inference.py sample_circle_160.jpg --conf 0.35
  python inference.py sample_six_160.jpg --conf 0.35
Expected IDs: 40 and 16 respectively.

This candidate adds black-circle distance and background variation using
scene-separated uploaded screenshots and existing training sources. Includes
matching-background sixes, E symbols, other classes and blank cards.
Existing lowres160 and circle_v2 packages are preserved.

Validation figures and limitations are in validation.json. Generated variants
are not independent captures. Screenshots include annotation pixels, so test
live Pi camera frames before deciding to replace the deployed model.
The packaged Python runner uses the same confidence/dedup/largest-box behavior
as v2. Optional C/L/R filtering was not evaluated here.
''')
    manifest={'source_checkpoint':str(checkpoint),'source_checkpoint_sha256':digest(checkpoint),'network_imgsz':640,
              'candidate_only':True,'requires_live_pi_validation':True,'sha256':{p.name:digest(p) for p in sorted(package.iterdir()) if p.is_file()}}
    (package/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    archive=ROOT/'handoff/mdp_circle_v3_ncnn.zip'
    with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED) as z:
        for p in sorted(package.iterdir()):
            if p.is_file():z.write(p,arcname=str(Path(package.name)/p.name))
    with zipfile.ZipFile(archive) as z:assert z.testzip() is None
    print('PACKAGED',str(archive),flush=True)
if __name__=='__main__':main()
