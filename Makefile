BLENDER := /Applications/Blender.app/Contents/MacOS/Blender
BLEND   := build/rcjet.blend
SAMPLES ?= 128

.PHONY: all build verify render export stl clean

all: build verify render export

build:
	$(BLENDER) --background --python plane/assemble.py

verify:
	python3 plane/verify.py

render:
	$(BLENDER) -b $(BLEND) -P plane/render.py -- all $(SAMPLES)

hero:
	$(BLENDER) -b $(BLEND) -P plane/render.py -- hero $(SAMPLES)

export:
	$(BLENDER) -b $(BLEND) -P plane/export.py -- glb

stl:
	$(BLENDER) -b $(BLEND) -P plane/export.py -- stl

clean:
	rm -rf build renders
