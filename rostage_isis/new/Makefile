GIT_DIR = build/stage
GIT_URL = https://github.com/rtv/Stage.git

default: clean
	mkdir -p $(GIT_DIR)
	cd $(GIT_DIR) && git clone $(GIT_URL)
	make only_build

only_build:	
	export STG=$(HOME)/ros_ws/src/rostage_isis
	cd $(GIT_DIR) && cmake -DCMAKE_INSTALL_PREFIX=$(STG) Stage
	cp build/CMakeLists.txt build/stage/Stage/
	cd $(GIT_DIR) && make -j2 
	cd $(GIT_DIR) && make install
	
clean: 
	rm -rf build/stage
	
