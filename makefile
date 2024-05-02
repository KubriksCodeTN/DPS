CCFLAGS = -std=c++20 -O3
CUDAFLAGS = -arch=native
CUDA_BIN = bin/dubins_cuda
SEQ_BIN = bin/dubins_seq

COMMON_OBJS = obj/main.o obj/planner.o obj/visilibity.o
CUDA_OBJ = $(COMMON_OBJS) obj/dubins_cuda.cu.o 

sequential: $(SEQ_BIN)
cuda: $(CUDA_BIN)

$(SEQ_BIN): $(COMMON_OBJS) obj/dubins_seq.o
	g++ $(CCFLAGS) $^ -o $@

$(CUDA_BIN): $(CUDA_OBJ)
	nvcc $(CUDAFLAGS) $(CCFLAGS) $^ -o $@

obj/%.o: src/%.cpp
	g++ $(CCFLAGS) -c $< -o $@

obj/%.o: src/visilibity/%.cpp
	g++ $(CCFLAGS) -c $< -o $@

obj/%.o: src/sequential/%.cpp
	g++ $(CCFLAGS) -c $< -o $@

obj/%.cu.o: src/cuda/%.cu
	nvcc $(CUDAFLAGS) $(CCFLAGS) -dc $< -o $@

makedir:
	mkdir obj bin

clean:
	rm obj/* bin/*