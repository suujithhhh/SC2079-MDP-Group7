"""Select v3 using the one-to-many head exported by NCNN, not default one-to-one."""
import argparse,json,shutil
from pathlib import Path
import cv2,torch
from train_circle import evaluate,gates
from train_circle_v3 import ROOT,DATA,RUN,REVIEW,BASE,paths,brief,rank

def cached(weights,items,dest):
    if dest.exists():
        r=json.loads(dest.read_text())
        assert r['requested_end2end'] is False
        assert [x['path'] for x in r['records']]==[str(p) for p in items]
        return r
    return evaluate(weights,items,dest,'0',32,end2end=False)

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--final',action='store_true');args=parser.parse_args()
    cv2.setNumThreads(1);torch.set_num_threads(4)
    legacy=[Path(p) for p in (ROOT/'circle_data_v2/selection_val.txt').read_text().splitlines()]
    target=paths('target_val')+paths('real_eval')
    base=cached(BASE,legacy,REVIEW/'deploy_baseline_legacy.json')
    initial=cached(BASE,target,REVIEW/'deploy_baseline_target.json');old=brief(initial)
    epochs=json.loads((REVIEW/'epoch_scores.json').read_text())
    results={}
    for e in epochs:
        epoch=e['epoch'];checkpoint=RUN/'weights'/f'epoch{epoch-1}.pt'
        if not checkpoint.exists():continue
        name=f'epoch_{epoch:02d}'
        new=cached(checkpoint,target,REVIEW/f'deploy_{name}_target.json');info=brief(new)
        replay=cached(checkpoint,legacy,REVIEW/f'deploy_{name}_legacy.json');legacy_check=gates(base,replay)
        false=lambda r:sum(x['returned']=='40' and x['expected']!='40' for x in r['records'])
        native=info['native_predictions']
        actual=[r for r in native if Path(r['path']).name.startswith(('capture_00_','capture_01_'))]
        extra={'native_real_improved':info['native_real_correct']>old['native_real_correct'],
               'original_failures_correct':len(actual)==2 and all(r['returned']=='40' for r in actual),
               'new_circle_not_worse':info['target']['40']['correct_return']>=old['target']['40']['correct_return'],
               'new_six_not_worse':info['target']['16']['correct_return']>=old['target']['16']['correct_return'],
               'new_false_circles_not_worse':false(new)<=false(initial)}
        results[name]={'checkpoint':str(checkpoint),'epoch':epoch,'summary':info,'legacy_check':legacy_check,
                       'target_checks':extra,'passed':legacy_check['passed'] and all(extra.values())}
        print('DEPLOY_EPOCH',epoch,'real',info['native_real_correct'],'reduced',info['reduced_real_correct'],
              'circle',info['target']['40']['correct_return'],'passed',results[name]['passed'],flush=True)
    eligible=[n for n,r in results.items() if r['passed']]
    selected=max(eligible,key=lambda n:rank(results[n]['summary'])) if eligible else None
    result={'baseline':old,'candidates':results,'selected':selected,'selection_head':'end2end=False (NCNN one-to-many)',
            'final':args.final,'production_model_replaced':False,'run':str(RUN),
            'real_screenshots_used_for_checkpoint_selection':True,'original_test_used_for_selection':False}
    (REVIEW/'deploy_selection.json').write_text(json.dumps(result,indent=2)+'\n')
    if args.final and selected:
        shutil.copy2(results[selected]['checkpoint'],RUN/'weights/candidate.pt')
    print('DEPLOY_SELECTION',selected,'final',args.final,flush=True)
if __name__=='__main__':main()
