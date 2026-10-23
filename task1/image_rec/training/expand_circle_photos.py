"""Extend the procedural circle dataset with photographed background surfaces."""
from collections import Counter
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import yaml

from circle_data import (ROOT, SEED, NAMES, TARGETS, labels, label_for, scene_key,
                         notebook_geometry, choose_sources, other_sources, read_source,
                         render, save_preview)


def photo_bank(split):
    rng=np.random.default_rng(SEED+(1 if split=="valid" else 0))
    paths=sorted((ROOT/split/"images").glob("*.jpg"))
    selected=rng.choice(len(paths),min(180,len(paths)),replace=False)
    bank=[]
    for idx in selected:
        path=paths[int(idx)];im=cv2.imread(str(path));h,w=im.shape[:2]
        occupied=np.zeros((h,w),np.uint8)
        for cls,poly in labels(label_for(path)):
            pts=np.rint(poly*[w,h]).astype(np.int32)
            x,y,bw,bh=cv2.boundingRect(pts)
            cv2.rectangle(occupied,(x-bw//2,y-bh//2),(x+3*bw//2,y+3*bh//2),255,-1)
        for _ in range(5):
            side=int(rng.integers(72,190));x=int(rng.integers(0,w-side));y=int(rng.integers(0,h-side))
            if occupied[y:y+side,x:x+side].any():continue
            patch=im[y:y+side,x:x+side]
            if patch.std()<12:continue
            bank.append((patch,{"image":str(path),"crop_xywh":[x,y,side,side]}))
    if split=="train":
        # Small glyph-free corners of actual patterned cards in the existing real photo.
        path=ROOT/"real /e8ca3def-5e13-4a11-9720-3b395c26cb2a.jpeg"
        im=cv2.imread(str(path))
        for name,box in [("wood_chip",(286,190,40,30)),("knit",(570,190,40,30)),
                         ("grid",(325,450,40,20)),("denim",(540,450,35,25)),
                         ("glitter",(670,743,40,25))]:
            x,y,w,h=box
            bank.append((im[y:y+h,x:x+w],{"image":str(path),"crop_xywh":list(box),"material":name}))
    if not bank:raise ValueError("No clean photographic textures")
    return bank


def main():
    old=ROOT/"circle_data_v1";out=ROOT/"circle_data_v2"
    if out.exists():raise FileExistsError(out)
    out.mkdir()
    manifest={"complete":False,"parent":str(old),"seed":SEED,"limitations":[
        "Synthetic validation only; actual failing Pi circle frames have not been supplied.",
        "Legacy source scenes may contain near duplicates; no claim of independent camera accuracy."]}
    (out/"manifest.json").write_text(json.dumps(manifest,indent=2))
    cv2.setNumThreads(1)
    mask_fn,quad_fn=notebook_geometry()
    val_pool,_=choose_sources("valid",mask_fn)
    val_keys={scene_key(p) for ps in val_pool.values() for p in ps}
    train_pool,_=choose_sources("train",mask_fn)
    train_pool={c:[p for p in ps if scene_key(p) not in val_keys] for c,ps in train_pool.items()}
    records=[json.loads(s) for s in (old/"samples.jsonl").read_text().splitlines()]
    added=[];previews=[];banks={}
    for split,pool,quantities in [("train",train_pool,{29:200,5:100,"other":50,"blank":100}),
                                  ("val",val_pool,{29:50,5:50,"other":28,"blank":50})]:
        bank=photo_bank("train" if split=="train" else "valid")
        banks[split]=[meta for _,meta in bank]
        if split=="train":
            save_preview([(meta.get("material","surface"),patch) for patch,meta in bank[-5:]],out/"real_texture_crops.jpg")
        (out/split/"images").mkdir(parents=True);(out/split/"labels").mkdir()
        for category,count in quantities.items():
            sources=pool[category] if isinstance(category,int) else other_sources(pool) if category=="other" else pool[29]+pool[5]
            for i in range(count):
                rng=np.random.default_rng(SEED+200000+(100000 if split=="val" else 0)
                                         +(category if isinstance(category,int) else 40 if category=="other" else 50)*1000+i)
                source=sources[i%len(sources)]
                image,cls,points,mask=read_source(source,mask_fn)
                # Half of training examples use actual card materials; half use scene surfaces.
                index=len(bank)-5+(i%5) if split=="train" and i%2==0 else int(rng.integers(len(bank)))
                patch,meta=bank[index]
                rendered=render(image,points,mask,rng,"photo",quad_fn,category=="blank",[patch])
                for size in (640,160):
                    target=out/split/"images"/f"photo_{category}_{i:05d}_{source.stem}_r{size}.jpg"
                    quality=int(rng.integers(75,96))
                    resized=cv2.resize(rendered,(size,size),interpolation=cv2.INTER_AREA)
                    if not cv2.imwrite(str(target),resized,[cv2.IMWRITE_JPEG_QUALITY,quality]):raise OSError(target)
                    label_for(target).write_text("" if category=="blank" else label_for(source).read_text())
                    added.append({"path":str(target),"source":str(source),"split":split,
                                  "scene":scene_key(source),"category":str(category),
                                  "image_id":None if category=="blank" else NAMES[cls],
                                  "texture":"photo","photo_source":meta,"size":size,"jpeg_quality":quality})
                if split=="train" and i<4:
                    x,y,w,h=cv2.boundingRect(points);pad=max(w,h)//2
                    previews.append((f"{NAMES[cls] if category!='blank' else 'empty'} / photo",rendered[max(0,y-pad):y+h+pad,max(0,x-pad):x+w+pad]))
            print(f"generated {split} photographic {category}: {count*2}",flush=True)
    records+=added
    train_paths=(old/"train.txt").read_text().splitlines()+[r["path"] for r in added if r["split"]=="train"]
    selection=(old/"selection_val.txt").read_text().splitlines()+[r["path"] for r in added if r["split"]=="val"]
    stress=[r["path"] for r in records if r["split"]=="val"]
    for name,paths in [("train",train_paths),("selection_val",selection),("stress_val",stress)]:
        assert len(paths)==len(set(paths))
        (out/f"{name}.txt").write_text("\n".join(paths)+"\n")
    for name,val in [("data","selection_val"),("stress","stress_val")]:
        config={"path":str(out),"train":str(out/"train.txt"),"val":str(out/f"{val}.txt"),
                "test":str(ROOT/"test/images"),"nc":31,"names":NAMES}
        (out/f"{name}.yaml").write_text(yaml.safe_dump(config,sort_keys=False))
    train_groups={r["scene"] for r in records if r["split"]=="train"}
    val_groups={r["scene"] for r in records if r["split"]=="val"}
    assert not train_groups&val_groups
    (out/"samples.jsonl").write_text("".join(json.dumps(r)+"\n" for r in records))
    (out/"texture_bank.json").write_text(json.dumps(banks,indent=2)+"\n")
    save_preview(previews,out/"photo_preview.jpg")
    manifest.update(complete=True,training_images=len(train_paths),selection_validation_images=len(selection),
                    new_generated_by_category=dict(Counter(f"{r['split']}/{r['category']}" for r in added)),
                    all_generated_by_category=dict(Counter(f"{r['split']}/{r['category']}" for r in records)),
                    source_split_overlap=0,expander_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    (out/"manifest.json").write_text(json.dumps(manifest,indent=2)+"\n")
    print(json.dumps(manifest,indent=2),flush=True)


if __name__=="__main__":main()
