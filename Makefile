run: 
	make build && make start
start: 
	./wvm ./hello-world.wasm
build: 
	g++ src/*.cc -o wvm -pthread -std=c++17