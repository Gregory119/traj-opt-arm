#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>

#include "so101_bus.hpp"


static void do_scan(int fd) {
    std::printf("scanning IDs \n");
    bool any = false;
    for (int id = 1; id <= 253; ++id) {
        if (so101::feetech_ping(fd, static_cast<uint8_t>(id), 10)) {
            std::printf("  found servo ID %d\n", id);
            any = true;
        }
        usleep(2000);
    }
    if (!any) std::printf("  no reply\n");
}

int main(int argc, char** argv) {
    
    if (argc != 2 && argc != 4) { //incorrectly formatted input args
        return 2;
    }
    
    const char* device = argv[1]; 
    int fd = so101::open_port_1Mbps(device); //open port and confgure
    
    if (fd < 0) { //error code on failed port configuration set
        std::fprintf(stderr, "cant open %s: %s\n", device, std::strerror(errno)); //print error message and error number
        return 1;
    }

    if (argc == 2) { 
        do_scan(fd); //scan for IDs using file descriptor
        ::close(fd); // close the port after scan
        return 0;
    }

    // re address
    uint8_t old_id = 0, new_id = 0; // initialize old and new id at 0

    if (old_id == new_id) { //error if ids are the same
        //std::fprintf(stderr, "old_id and new_id are the same\n");
        ::close(fd); // close port
        return 2;
    }

    // stop if new id already responds
    if (so101::feetech_ping(fd, new_id, 15)) {
        //std::fprintf(stderr,
            //"a servo already replied at new_id=%u.\n"
        ::close(fd);
        return 1;
    }

    // find servo at old_id
    if (!so101::feetech_ping(fd, old_id, 20)) {
        //std::fprintf(stderr, "Error: no servo replied at old_id=%u.\n", (unsigned)old_id);
        //std::fprintf(stderr, "Scanning to show what *is* present:\n");
        do_scan(fd); //scan at file descriptor location
        ::close(fd); // close port
        return 1;
    }
    
    std::printf("Changing servo ID %u -> %u ...\n", (unsigned)old_id, (unsigned)new_id); //performing re addressing

    // control table at 0x05
    const uint8_t ADDR_ID = 0x05;
    // eeprom address
    const uint8_t LOCK_ADDR = 0x37;
    // unlock eeprom
    so101::feetech_write_byte(fd, old_id, LOCK_ADDR, 0);

    if (!so101::feetech_write_byte(fd, old_id, ADDR_ID, new_id, 80)) { //write the new servo ID to the old ID
        std::fprintf(stderr, 
            "write to ID register failed \n");
        ::close(fd);
        return 1;
    }
    // lock eeprom again after re addressing
    so101::feetech_write_byte(fd, new_id, LOCK_ADDR, 1);


    // wait for servo to apply new ID
    usleep(100000);

    if (!so101::feetech_ping(fd, new_id, 30)) { //double check rewrite with ping to new ID
        std::fprintf(stderr,
            "ID written but new_id=%u did not reply to ping\n", //no reply received
            (unsigned)new_id);
        ::close(fd); //close port
        return 1;
    }

    std::printf("servo now responds at ID %u.\n", (unsigned)new_id); //confirmation message

    ::close(fd); //close port
    return 0;
}
