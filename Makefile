EXEC = acd_server
OBJS = ext_dev.o camera.o acd_server.o acd_proto_v4.o acd_stmio.o network.o fam_setup.o rtc.o
LDLIBS = -lpthread  
 
all: $(EXEC)
 
$(EXEC): $(OBJS)
	$(CXX) -O3 $(LDFLAGS) -o $@ $(OBJS) $(LDLIBS)


romfs:
	$(ROMFSINST)    /bin/$(EXEC)


clean:
	rm -f $(EXEC) *.elf *.gdb *.o