SHELL = /bin/sh

.SUFFIXES:


all : libSPAnalysis.so

libSPAnalysis.so : StrongPointAnalysis.o Cluster.o
	$(CXX) -shared $^ -o $@ $(LDFLAGS) -lm
	chcon -t texrel_shlib_t $@

StrongPointAnalysis.o : StrongPointAnalysis.C StrongPointAnalysis.h Cluster.h
	$(CXX) -c $< -o $@ -O3 $(CXXFLAGS)

Cluster.o : Cluster.C Cluster.h
	$(CXX) -c $< -o $@ $(CXXFLAGS)

clean :
	-rm -f libSPAnalysis.so StrongPointAnalysis.o Cluster.o
