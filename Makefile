run: 
	make build && ./wvm ./hello-world.wasm
build: 
	g++ src/*.cc -o wvm -pthread -std=c++17