SOURCES = xyz.ml acft.ml simu.ml
INCLUDES = -I +labltk
LIBS = labltk.cma unix.cma
TARGET = simu

OCAMLC   = ocamlc -g
OCAMLOPT = ocamlopt.opt
OCAMLDEP = ocamldep

LIBS_OPT = $(LIBS:.cma=.cmxa)

OBJS = $(SOURCES:.ml=.cmo)
OBJS_OPT = $(SOURCES:.ml=.cmx)

all: .depend opt
byte: $(TARGET).byte
opt: $(TARGET)

$(TARGET).byte: $(OBJS)
	$(OCAMLC) -o $@ $(INCLUDES) $(LIBS) $^

$(TARGET): $(OBJS_OPT)
	$(OCAMLOPT) -o $@ $(INCLUDES) $(LIBS_OPT) $^

%.cmi: %.mli
	$(OCAMLC) $(INCLUDES) $<

%.cmo: %.ml
	$(OCAMLC) $(INCLUDES) -c $<

%.cmx: %.ml
	$(OCAMLOPT) $(INCLUDES) -c $<

clean:
	rm -f *.cm[iox] *~ *.o

cleanall: clean
	$(TARGET) $(TARGET).opt

.depend: $(SOURCES)
	$(OCAMLDEP) *.mli *.ml > .depend

include .depend
