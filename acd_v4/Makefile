EXEC = acd_server
OBJS = ext_dev.o camera.o acd_server.o  
LDLIBS = -lpthread 
 
all: $(EXEC)
 
$(EXEC): $(OBJS)
	$(CXX) -O3 $(LDFLAGS) -o $@ $(OBJS) $(LDLIBS)


romfs:
	$(ROMFSINST)    /bin/$(EXEC)


clean:
	rm -f $(EXEC) *.elf *.gdb *.o