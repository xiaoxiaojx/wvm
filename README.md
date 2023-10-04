# wvm

A toy WebAssembly virtual machine.
> It is mainly used for learning 📖 ✍️, Inspired by [twvm](https://github.com/Becavalier/twvm) and [WebAssemblyToolkit](https://github.com/yaozhongxiao/WebAssemblyToolkit).


## Getting Started
```bash
make build && ./wvm ./hello-world.wasm
```
```bash
-> parse
 -> parseMagicNumber: [ 0 97 115 109 ]
 -> parseVersion: [ 1 0 0 0 ]
 -> parseSection
  -> parseSectionId: 1
  -> parseSectionId: 3
  -> parseSectionId: 7
  -> parseSectionId: 10
-> instantiate
 -> func
 -> entryFuc: add
-> execute
 -> call: 0,paramCount: 2
  -> local.get: 0
  -> local.get: 1
 -> call: 1,paramCount: 2
  -> local.get: 0
  -> local.get: 1
  -> op.I32Add
   -> x: 2,y: 1
   -> ret: 3
ret: 3
```