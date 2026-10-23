"""Summarize official metrics and a separate visual audit of missing ID labels."""
import csv,json,copy,hashlib
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from retest_circle_v3_all import OUT,ROOT,metric,NAMES,SYMBOL

def read_reports(condition,tag):
    p=OUT/f'{tag}_{condition}.json'
    if p.exists():return json.loads(p.read_text())
    if tag=='v2' and condition=='test_native':return json.loads((ROOT/'runs/yolo26s_seg_mdp_circle_v2_review/test_candidate_ncnn.json').read_text())
    if tag=='v2' and condition=='validation_160':
        d=json.loads((ROOT/'runs/yolo26s_seg_mdp_circle_v2_review/candidate_ncnn.json').read_text())
        d['records']=[r for r in d['records'] if '/lowres_data_160/val/' in r['path']];return d
    raise FileNotFoundError(p)

def main():
    raw=json.loads((OUT/'summary.json').read_text());audits=json.loads((OUT/'label_audit.json').read_text())['entries']
    lookup={(r['split'],r['stem']):r['visually_observed_image_id'] for r in audits}
    for r in audits:assert hashlib.sha256(Path(r['image']).read_bytes()).hexdigest()==r['image_sha256']
    audited={};csvrows=[];differences=[]
    for condition in raw['summary']:
        split='valid' if condition.startswith('validation') else 'test';audited[condition]={};all_rows={}
        for tag in ['v2','v3']:
            rows=copy.deepcopy(read_reports(condition,tag)['records'])
            for r in rows:
                if r['expected'] is None:
                    stem=Path(r['path']).stem
                    if condition=='validation_160':
                        assert stem.startswith('s0_') and stem.endswith('_r160');stem=stem[3:-5]
                    r['original_dataset_expected']=None;r['expected']=lookup[(split,stem)]
            info=metric(rows)
            # There are no replacement polygons: never report revised localization/AP.
            for v in info['per_class'].values():v.pop('localized_correct',None)
            audited[condition][tag]=info;all_rows[tag]=rows
        for ident in NAMES:
            a,b=audited[condition]['v2']['per_class'][ident],audited[condition]['v3']['per_class'][ident]
            csvrows.append({'condition':condition,'image_id':ident,'symbol':SYMBOL[ident],'images':b['images'],
              'v2_correct':a['correct'],'v3_correct':b['correct'],'correct_change':b['correct']-a['correct'],
              'v3_recall':b['recall'],'v3_precision':b['precision'],'v3_f1':b['f1'],'v3_missed':b['missed'],'v3_wrong_id':b['wrong_id']})
        for old,new in zip(all_rows['v2'],all_rows['v3']):
            assert old['path']==new['path']
            if old['returned']!=new['returned']:
                differences.append({'condition':condition,'path':old['path'],'expected_after_visual_audit':new['expected'],
                   'v2_returned':old['returned'],'v3_returned':new['returned']})
    (OUT/'audited_summary.json').write_text(json.dumps({'label_files_modified':False,'audit_scope':'Classification IDs only for all19 original empty-labeled images; no corrected boxes/masks.',
        'results':audited,'prediction_changes':differences},indent=2)+'\n')
    with (OUT/'all_targets_audited.csv').open('w') as f:
        w=csv.DictWriter(f,fieldnames=list(csvrows[0]));w.writeheader();w.writerows(csvrows)
    conditions=list(audited);matrix=np.array([[audited[c]['v3']['per_class'][i]['recall']*100 for c in conditions] for i in NAMES])
    fig,ax=plt.subplots(figsize=(10,13));im=ax.imshow(matrix,vmin=50,vmax=100,cmap='RdYlGn',aspect='auto')
    ax.set_xticks(range(4),['Validation\nnative','Validation\n160px','Test\nnative','Test\n160px'])
    ax.set_yticks(range(31),[f'{i}: {SYMBOL[i]}' for i in NAMES]);ax.tick_params(labelsize=9)
    for y,ident in enumerate(NAMES):
        for x,c in enumerate(conditions):
            s=audited[c]['v3']['per_class'][ident];ax.text(x,y,f'{s["correct"]}/{s["images"]}',ha='center',va='center',fontsize=8,color='black')
    ax.set_title('V3 NCNN: correct returned IDs per target\nSeparate visual audit supplies19 missing IDs; original labels unchanged',fontsize=11)
    fig.colorbar(im,ax=ax,label='ID recall (%)',shrink=.6);fig.tight_layout();fig.savefig(OUT/'all_targets_v3.png',dpi=150);plt.close(fig)
    lines=['V3 all-target retest','====================','',
      'Both models: NCNN, network640, confidence0.35, class-agnostic IoU0.5 dedup,',
      'largest returned box, no C/L/R filter. All31 IDs are present in every set.',
      'V3 was freshly evaluated on2,226 inputs:778 validation+335 test, each at',
      'native and160 resolution. Resolution pairs are not independent images.',
      'Unchanged v2 baselines for test-native and validation160 were reused with',
      'model hashes verified; v2 native-validation and test160 were freshly run.','',
      'Existing dataset-label results (correct returns / images):']
    for c,models in raw['summary'].items():lines.append(f'  {c}: v2 {models["v2"]["correct_returns"]}/{models["v2"]["images"]}; v3 {models["v3"]["correct_returns"]}/{models["v3"]["images"]}')
    lines+=['','Annotation audit:',
      'Every originally empty-labeled image contains a visible target:7 test',
      'arrows;11 validation arrows and1 validation X. See label_audit.csv and',
      'the audit image galleries. Missing annotations can make a detected arrow',
      'look like a false positive, and a missed arrow look correct. Original',
      'label files were NOT changed. The following secondary results replace',
      'only the expected image IDs for these19 visually inspected images.',
      'No replacement polygons/masks were made, and AP was not recomputed with',
      'these ID-only corrections.','',
      'Classification after the separate visual ID audit:']
    for c,models in audited.items():
        a,b=models['v2'],models['v3'];lines.append(f'  {c}: v2 {a["correct_returns"]}/{a["images"]} ({a["overall_accuracy"]:.2%}); v3 {b["correct_returns"]}/{b["images"]} ({b["overall_accuracy"]:.2%})')
    lines+=['','Per-target v3 results after the visual ID audit:','ID  Symbol                         Val native   Val160   Test native   Test160']
    for ident in NAMES:
        cells=[f'{audited[c]["v3"]["per_class"][ident]["correct"]}/{audited[c]["v3"]["per_class"][ident]["images"]}' for c in conditions]
        lines.append(f'{ident:6s}{SYMBOL[ident]:30s}'+''.join(f'{cell:>12s}' for cell in cells))
    lines+=['','Changed predictions (v2 -> v3):']
    for r in differences:lines.append(f'  {r["condition"]}: {Path(r["path"]).name}; expected {r["expected_after_visual_audit"]}; {r["v2_returned"]} -> {r["v3_returned"]}')
    standard=json.loads((OUT/'standard_metrics.json').read_text())
    lines+=['','Standard box/mask AP (PyTorch CUDA with NCNN-matching end2end=False;',
            'original dataset annotations, confidence0.001 for PR curves; not the',
            'same metric as fixed0.35 returned-ID accuracy):']
    for c,models in standard['results'].items():
        for tag,r in models.items():
            m=r['metrics'];lines.append(f'  {c} {tag}: box AP50={m["metrics/mAP50(B)"]:.2%}, box AP50-95={m["metrics/mAP50-95(B)"]:.2%}, mask AP50-95={m["metrics/mAP50-95(M)"]:.2%}')
    lines+=['','Interpretation:',
      'The original test and reduced validation ID comparisons show whether',
      'circle retraining preserved other classes. Native-resolution validation',
      'has two additional up-arrow misses in v3 that the aggregate raw score',
      'hides because one image lacks its annotation. All changes are listed',
      'above. Masks are slightly worse on standard AP; v3 is not a universal',
      'improvement in every metric. Arrow labeling and difficult arrow views',
      'are the next data issues to address before further broad retraining.',
      'The earlier circle-specific evaluation remains in the v3 review folder.',
      'Legacy datasets contain near duplicates and validation was used during',
      'training/selection. Pi latency and live-camera performance were not',
      'measured by this host-CPU retest. No model weights or Pi packages changed.']
    (OUT/'REPORT.txt').write_text('\n'.join(lines)+'\n')
    print('\n'.join(lines[:36]));print('REPORT_READY',str(OUT/'REPORT.txt'))
if __name__=='__main__':main()
