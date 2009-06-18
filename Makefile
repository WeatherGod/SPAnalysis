SHELL = /bin/sh

.SUFFIXES:


all : libSPAnalysis.so

libSPAnalysis.so : StrongPointAnalysis.o Cluster.o
	$(CXX) -shared $^ -o $@ $(LDFLAGS) -lm
#	chcon -t texrel_shlib_t $@

StrongPointAnalysis.o : StrongPointAnalysis.C StrongPointAnalysis.h Cluster.h
	$(CXX) -c $< -o $@ -O3 $(CXXFLAGS)

Cluster.o : Cluster.C Cluster.h
	$(CXX) -c $< -o $@ $(CXXFLAGS)

install : libSPAnalysis.so StrongPointAnalysis.h Cluster.h
	cp libSPAnalysis.so ~/lib
	cp StrongPointAnalysis.h ~/include
	cp Cluster.h ~/include

clean :
	-rm -f libSPAnalysis.so StrongPointAnalysis.o Cluster.o
