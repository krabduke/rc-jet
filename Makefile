BLENDER := /Applications/Blender.app/Contents/MacOS/Blender
BLEND   := build/rcjet.blend
SAMPLES ?= 128

.PHONY: all build verify render export stl manifest viewer validate aero clean

all: build verify render export

build:
	$(BLENDER) --background --python plane/assemble.py
	python3 tools/make_manifest.py

verify:
	python3 plane/verify.py
	python3 tools/audit_structure.py
	python3 tools/audit_geometry.py
	python3 tools/audit_intersect.py
	python3 tools/audit_duct.py
	python3 tools/audit_fit.py
	python3 tools/audit_manifest.py
	python3 tools/check_vendor.py
	node tools/check_panels.mjs
	node tools/validate_viewer.mjs .

render:
	$(BLENDER) -b $(BLEND) -P plane/render.py -- all $(SAMPLES)

hero:
	$(BLENDER) -b $(BLEND) -P plane/render.py -- hero $(SAMPLES)

export:
	$(BLENDER) -b $(BLEND) -P plane/export.py -- glb

stl:
	$(BLENDER) -b $(BLEND) -P plane/export.py -- stl

manifest:
	python3 tools/make_manifest.py

viewer: 
	@echo "Serving http://localhost:8788/viewer/ - Ctrl-C to stop"
	@python3 -m http.server 8788 --bind 127.0.0.1

clean:
	rm -rf build renders

# numpy lives in Blender's bundled Python, not the system one
BPY := /Applications/Blender.app/Contents/Resources/5.2/python/bin/python3.13

validate:
	$(BPY) aero/validate.py

aero:
	$(BPY) aero/analyse.py

vendor-viewer:
	python3 tools/vendor_viewer.py
