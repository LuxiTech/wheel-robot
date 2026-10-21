"""Lazy adapters to upstream model APIs. No simulated predictions on failure."""
from pathlib import Path
from contextlib import contextmanager
import sys
import numpy as np


@contextmanager
def cpu_tensor_defaults():
    """FoundationPose changes Torch's global default to CUDA during register.

    NanoSAM builds CPU image-normalization tensors. Isolate that global side
    effect in this serial inference worker, then restore FP's prior default.
    """
    import torch
    previous=torch.tensor([]).type()
    torch.set_default_tensor_type(torch.FloatTensor)
    try:
        yield
    finally:
        torch.set_default_tensor_type(previous)


def required(path):
    if not path or not Path(path).is_file(): raise FileNotFoundError('Required model asset: '+str(path))
    return str(Path(path).resolve())


class NanoDetectorSegmenter:
    def __init__(self,owl_engine,sam_encoder,sam_decoder,prompt='a plastic crate',threshold=.15):
        assets=[required(p) for p in (owl_engine,sam_encoder,sam_decoder)]
        from nanoowl.owl_predictor import OwlPredictor
        from nanosam.utils.predictor import Predictor
        self.owl=OwlPredictor('google/owlvit-base-patch32',image_encoder_engine=assets[0])
        self.sam=Predictor(assets[1],assets[2]);self.prompt=[prompt];self.threshold=threshold
        self.text=self.owl.encode_text(self.prompt)

    def detect(self,rgb):
        from PIL import Image
        with cpu_tensor_defaults():
            output=self.owl.predict(image=Image.fromarray(rgb),text=self.prompt,
                                    text_encodings=self.text,threshold=self.threshold,pad_square=False)
        boxes=output.boxes.detach().cpu().numpy();scores=output.scores.detach().cpu().numpy()
        return [dict(box=b.tolist(),score=float(v)) for b,v in zip(boxes,scores)]

    def segment(self,rgb,box):
        from PIL import Image
        with cpu_tensor_defaults():
            self.sam.set_image(Image.fromarray(rgb))
            masks,_,_=self.sam.predict(np.asarray(box,float).reshape(2,2),np.array([2,3]))
        return (masks[0,0]>0).detach().cpu().numpy()


class FoundationPoseBackend:
    def __init__(self,repository,mesh,*,debug_dir):
        required(mesh)
        repo=Path(repository).resolve()
        if not (repo/'estimater.py').is_file(): raise FileNotFoundError('FoundationPose source required')
        sys.path.insert(0,str(repo))
        import trimesh
        import nvdiffrast.torch as dr
        from estimater import FoundationPose
        self.mesh=trimesh.load(mesh,force='mesh')
        if self.mesh.is_empty: raise ValueError('Empty object mesh')
        self.est=FoundationPose(model_pts=self.mesh.vertices,model_normals=self.mesh.vertex_normals,
                                mesh=self.mesh,debug=0,debug_dir=str(debug_dir),glctx=dr.RasterizeCudaContext())
        self.identity=None

    def register(self,model_input,mask,identity):
        self.identity=None
        t=self.est.register(K=model_input.k,rgb=model_input.rgb,depth=model_input.depth,ob_mask=mask,iteration=5)
        self.identity=identity
        return t

    def track(self,model_input,identity):
        if identity!=self.identity: raise ValueError('Tracking target identity changed')
        return self.est.track_one(rgb=model_input.rgb,depth=model_input.depth,K=model_input.k,iteration=2)
