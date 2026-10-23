"""Frozen source-group splits, arrow-focused additions, and unchanged v1 replay."""
from pathlib import Path
import json, os, hashlib, random
from collections import Counter, defaultdict
import cv2
import numpy as np
from arrow_augmentation import restyle, canonical_scene
ROOT=Path(__file__).resolve().parents[1]
OLD=ROOT.parent
SEED=261013

def main():
    random.seed(SEED);cv2.setNumThreads(1)
    old=json.loads((OLD/'data/manifest.json').read_text())
    source=json.loads((OLD/'audit/source_manifest.json').read_text())
    source_by_path={x['path']:x for x in source}
    records=[]; rng=np.random.default_rng(SEED)
    if (ROOT/'data/manifest.json').exists():raise FileExistsError('Dataset already built; preserve its frozen manifest.')
    for split in ['train','val','test']:
        for folder in ['images','labels']:(ROOT/'data'/split/folder).mkdir(parents=True,exist_ok=True)
    def add(split,kind,image,objects,groups,metadata):
        index=len(records);name=f'{kind}_{index:05d}'
        dst=ROOT/'data'/split/'images'/f'{name}.jpg'
        if isinstance(image,(str,Path)):
            os.symlink(Path(image).resolve(),dst)
        else:
            assert image.shape==(640,640,3)
            assert cv2.imwrite(str(dst),image,[cv2.IMWRITE_JPEG_QUALITY,95])
        lines=[]
        for obj in objects:
            x1,y1,x2,y2=map(float,obj['box']);assert 0<=x1<x2<=640 and 0<=y1<y2<=640,obj
            lines.append(f"{int(obj['class'])} {(x1+x2)/1280:.8f} {(y1+y2)/1280:.8f} {(x2-x1)/640:.8f} {(y2-y1)/640:.8f}")
        (ROOT/'data'/split/'labels'/f'{name}.txt').write_text('\n'.join(lines)+ ('\n' if lines else ''))
        records.append({'path':str(dst),'split':split,'kind':kind,'objects':objects,'groups':sorted(set(groups)),**metadata})
    for record in old['records']:
        add(record['split'],'regression_'+record['kind'],record['path'],record['objects'],record['groups'],{'source':record['source'],'v1_path':record['path']})
    for split in ['train','val','test']:
        chosen=[r for r in old['records'] if r['split']==split and r['kind']=='real' and any(o['class'] in (1,2) for o in r['objects'])]
        for r in chosen:
            s=source_by_path[r['source']];original=cv2.imread(s['path'])
            for variant in range(2 if split=='train' else 1):
                transformed,objects=restyle(original,s,rng)
                add(split,'white_card_restyle',transformed,objects,[s['group']],{'source':s['path'],'variant':variant,'synthetic':True})
                if split=='train' and variant==0:
                    mirrored=cv2.flip(transformed,1)
                    objs=[{'class':3-o['class'],'box':[640-o['box'][2],o['box'][1],640-o['box'][0],o['box'][3]]} for o in objects]
                    add(split,'white_card_mirrored',mirrored,objs,[s['group']],{'source':s['path'],'synthetic':True,'class_swap':'left1 <-> right2'})
        negatives=[s for s in source if s['split']==split and s['old_class'] not in (27,28,30)]
        random.Random(SEED+len(split)).shuffle(negatives)
        n=2400 if split=='train' else 360
        sizes=[16,20,24,32,48,64,96,128]
        p=[.12,.14,.16,.16,.16,.12,.08,.06]
        for i in range(n):
            cls=1+i%2;s=negatives[i%len(negatives)]
            size=float(rng.choice(sizes,p=p)) if split=='train' else float(sizes[(i//2)%len(sizes)])
            if split=='train':size*=rng.uniform(.9,1.12)
            im,objs=canonical_scene(cv2.imread(s['path']),rng,cls,size,holdout=split!='train')
            add(split,'white_card_canonical',im,objs,[s['group']],{'background_source':s['path'],'nominal_target_px':size,'synthetic':True,'renderer':'train_polygon' if split=='train' else 'holdout_composite'})
        for i in range(400 if split=='train' else 120):
            s=negatives[(n+i)%len(negatives)]
            negative=['up','down','triangle','blank'][i%4]
            im,objs=canonical_scene(cv2.imread(s['path']),rng,1,float(rng.choice(sizes[1:])),holdout=split!='train',negative_kind=negative)
            assert objs==[]
            add(split,'white_card_negative',im,objs,[s['group']],{'background_source':s['path'],'negative_kind':negative,'synthetic':True})
    groups={sp:{g for r in records if r['split']==sp for g in r['groups']} for sp in ['train','val','test']}
    overlap={a+'_'+b:len(groups[a]&groups[b]) for a,b in [('train','val'),('train','test'),('val','test')]}
    assert not any(overlap.values()),overlap
    counts={sp:dict(Counter(r['kind'] for r in records if r['split']==sp)) for sp in groups}
    manifest={'seed':SEED,'classes':{'0':'bullseye','1':'left','2':'right'},'legacy_ids':{'0':'marker','1':'39','2':'38'},'source_manifest_sha256':hashlib.sha256((OLD/'audit/source_manifest.json').read_bytes()).hexdigest(),'group_overlap':overlap,'counts':counts,'records':records,'notes':['All new black-on-white images are synthetic or restyled, not real Task2 camera captures.','Train/val/test source groups remain disjoint. Existing v1 regression test has been evaluated previously.','No supplied arena reference image used for generation, checkpoint or threshold selection.','Horizontal mirrored training examples explicitly swap left/right labels.']}
    (ROOT/'data/manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    (ROOT/'data/data.yaml').write_text(f'path: {ROOT / "data"}\ntrain: train/images\nval: val/images\ntest: test/images\nnames:\n  0: bullseye\n  1: left\n  2: right\n')
    preview=[]
    for kind in ['white_card_restyle','white_card_mirrored','white_card_canonical','white_card_negative']:
        choices=[r for r in records if r['kind']==kind and r['split']=='train']
        for r in random.Random(88).sample(choices,6):
            im=cv2.imread(r['path'])
            for o in r['objects']:
                a,b,c,d=map(round,o['box']);cv2.rectangle(im,(a,b),(c,d),(0,255,0),2);cv2.putText(im,['bullseye','left','right'][o['class']],(a,max(14,b-4)),0,.5,(0,255,0),1)
            im=cv2.resize(im,(256,256));cv2.putText(im,kind.replace('white_card_',''),(5,250),0,.38,(0,0,255),1);preview.append(im)
    cv2.imwrite(str(ROOT/'reports/dataset_preview.jpg'),np.vstack([np.hstack(preview[i:i+6]) for i in range(0,len(preview),6)]))
    (ROOT/'reports/dataset_summary.json').write_text(json.dumps({k:v for k,v in manifest.items() if k!='records'},indent=2)+'\n')
    print(json.dumps(counts,indent=2),flush=True)
if __name__=='__main__':main()
