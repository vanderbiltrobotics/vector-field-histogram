
CC=g++
IDIR=include
SDIR=src
ODIR=obj

IFLAG=-I$(IDIR)

histogram_grid: $(ODIR)/HistogramGrid.o 
	$(CC) -o histogram_grid $(ODIR)/HistogramGrid.o

$(ODIR)/HistogramGrid.o : $(SDIR)/HistogramGrid.cpp $(IDIR)/HistogramGrid.h
	$(CC) -c $(IFLAG) $(SDIR)/HistogramGrid.cpp

clean:
	rm -f $(ODIR)/*.o *~ core $(INCDIR)/*~