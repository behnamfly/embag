
export CC=/usr/bin/gcc-10
export CXX=/usr/bin/g++-10	

cmake -S . -B build -G"Ninja Multi-Config"
cmake --build build/ --config Release






-DCMAKE_CUDA_COMPILER


export PATH="/usr/local/cuda-12.3/bin:$PATH"
export LD_LIBRARY_PATH="/usr/local/cuda-12.3/lib64:$LD_LIBRARY_PATH"
