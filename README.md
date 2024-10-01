python = 3.9

pip install bagpy open3d opencv-python easydict matplotlib numba h5py hdf5plugin mayavi configobj pyqt5 imageio torchsnooper dropblock PyOpenGL glfw imgui[glfw]

pip install numpy==1.26.4

conda install pytorch==1.11.0 torchvision==0.12.0 torchaudio==0.11.0 cudatoolkit=11.3 -c pytorch

pip install torch-scatter -f https://data.pyg.org/whl/torch-1.11.0%2Bcu113.html
pip install spconv-cu113



git clone https://github.com/astra-vision/PaSCo.git

conda create -y -n pasco python=3.9
conda activate pasco

conda install pytorch==1.11.0 torchvision==0.12.0 torchaudio==0.11.0 cudatoolkit=11.3 -c pytorch


conda install openblas-devel -c anaconda
export CUDA_HOME=/usr/local/cuda-11.3

git clone https://github.com/NVIDIA/MinkowskiEngine.git
cd MinkowskiEngine
python setup.py install --blas_include_dirs=${CONDA_PREFIX}/include --blas=openblas

pip install --no-cache-dir pytorch_lightning==1.9.0

cd PaSCo/
pip install -r requirements.txt

pip install torch-scatter -f https://data.pyg.org/whl/torch-1.11.0%2Bcu113.html

pip install -e ./



pre-trained
cd ckpt
wget https://github.com/astra-vision/PaSCo/releases/download/v0.1.0/pasco.ckpt


SSC-RS

conda create -n SSC python=3.8
conda activate SSC


conda install pytorch==1.11.0 torchvision==0.12.0 torchaudio==0.11.0 cudatoolkit=11.3 -c pytorch
pip install torch-scatter -f https://data.pyg.org/whl/torch-1.11.0%2Bcu113.html

pip install PyYAML dropblock