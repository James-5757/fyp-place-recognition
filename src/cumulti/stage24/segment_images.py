"""Real pretrained 2D segmentation only until optical mounting calibration resolves."""
import time,resource
import cv2,torch
from PIL import Image
from transformers import SegformerImageProcessor,SegformerForSemanticSegmentation
from common import *

def run():
    if (A/'segmentation_runtime.json').exists():raise RuntimeError('Refuse to overwrite segmentation pilot')
    torch.set_num_threads(2);torch.manual_seed(24);modelpath=Path('/home/cas/.cache/stage24_model')
    cfg=json.loads((modelpath/'config.json').read_text());labels=cfg['id2label']
    categories=['unknown','building','road/asphalt','grass','vegetation','vehicle','dirt/other']
    palette=np.array([[110,120,130],[210,170,110],[160,90,190],[100,220,65],[25,130,55],[240,75,65],[155,110,60]],np.uint8)
    taxonomy={}
    for i,name in labels.items():
        cat='building' if name in ['building','house','skyscraper'] else 'road/asphalt' if name in ['road','sidewalk','path','runway'] else 'grass' if name=='grass' else 'vegetation' if name in ['tree','plant','palm','flower','field'] else 'vehicle' if name in ['car','truck','bus','van','minibike','bicycle','boat','ship','airplane','train'] else 'dirt/other' if name in ['earth','sand','land','mountain','rock','hill'] else 'unknown'
        taxonomy[i]=dict(model_class=name,category=cat,category_id=categories.index(cat))
    policy=dict(model='nvidia/segformer-b0-finetuned-ade-512-512',revision=json.loads((modelpath/'revision.json').read_text())['revision'],
        weights_sha256=sha(modelpath/'model.safetensors'),threshold=.5,inference_size=[512,512],taxonomy=taxonomy,
        road_category_limitation='road/asphalt includes ADE sidewalk/path/runway; material asphalt is NOT verified',
        no_GT_or_LiDAR_labels=True,projection_status='BLOCKED_PENDING_COMPLETE_OPTICAL_MOUNTING_CHAIN',selection='six evenly indexed predeclared pilot RGB frames per robot; not selected by labels')
    policy['geometry_projection']='NOT_PERFORMED'
    dump(A/'semantic_model_policy.json',policy)
    processor=SegformerImageProcessor.from_pretrained(modelpath,local_files_only=True)
    model=SegformerForSemanticSegmentation.from_pretrained(modelpath,local_files_only=True);model.eval()
    man=json.loads((B/'camera_data_manifest.json').read_text());samples=[];dist=[];times=[]
    lut=np.array([taxonomy[str(i)]['category_id'] for i in range(len(labels))],np.uint8)
    for robot,data in man['robots'].items():
        frames=data['pilot_frames'];indices=np.linspace(0,len(frames)-1,6,dtype=int)
        for j in indices:
            f=frames[int(j)];image=Image.open(ROOT/f['file']).convert('RGB');start=time.monotonic()
            with torch.inference_mode():
                logits=model(**processor(images=image,return_tensors='pt')).logits
                logits=torch.nn.functional.interpolate(logits,size=(image.height,image.width),mode='bilinear',align_corners=False)
                confidence,classes=logits.softmax(1).max(1)
            prob=confidence[0].numpy();rawclass=classes[0].numpy();cat=lut[rawclass];cat[prob<.5]=0;times.append(time.monotonic()-start)
            name=f"{robot}_{f['rgb_message_index']:06d}";dest=SITE/'samples';dest.mkdir(parents=True,exist_ok=True)
            rgb=dest/(name+'_rgb.jpg');cv2.imwrite(str(rgb),np.array(image)[:,:,::-1],[cv2.IMWRITE_JPEG_QUALITY,85])
            mask=dest/(name+'_semantic.png');cv2.imwrite(str(mask),palette[cat][:,:,::-1])
            # Preserve raw class ID and actual confidence, server-only float arrays.
            arrays=A/'segmentation_arrays';arrays.mkdir(exist_ok=True)
            np.savez_compressed(arrays/(name+'.npz'),ADE_class=rawclass.astype(np.uint8),confidence=prob.astype(np.float32),category=cat)
            for i,label in enumerate(categories):dist.append(dict(scope='IMAGE_PIXELS_ONLY',robot_id=robot,sample=name,category=label,pixels=int((cat==i).sum()),fraction=float(np.mean(cat==i))))
            samples.append(dict(robot_id=robot,original_RGB_message_index=f['rgb_message_index'],camera_bag_timestamp_ns=f['camera_bag_timestamp_ns'],
                rgb='samples/'+rgb.name,semantic='samples/'+mask.name,source_RGB_sha256=f['sha256'],
                unknown_fraction=float(np.mean(cat==0)),mean_confidence=float(prob.mean()),seconds=times[-1],
                array_file=str((arrays/(name+'.npz')).relative_to(ROOT)),array_sha256=sha(arrays/(name+'.npz'))))
    write_csv(A/'semantic_class_distribution.csv',dist)
    dump(A/'segmentation_runtime.json',dict(status='2D_MODEL_EXECUTED_3D_PROJECTION_BLOCKED',samples=samples,inference_seconds=stats(times),
        process_peak_RSS_MiB=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss/1024,not_semantic_3D_mapping_success=True))
    print('SEGMENTED',len(samples),'real RGB images',flush=True)

if __name__=='__main__':run()
