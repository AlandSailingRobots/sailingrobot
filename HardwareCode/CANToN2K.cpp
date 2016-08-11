#include <iostream>

void CanIdToN2k(unsigned long id, unsigned char &prio, unsigned long &pgn, unsigned char &src, unsigned char &dst) {
  unsigned char PF = (unsigned char) (id >> 16);
  unsigned char PS = (unsigned char) (id >> 8);
  unsigned char DP = (unsigned char) (id >> 24) & 1;

    src = (unsigned char) id >> 0;
    prio = (unsigned char) ((id >> 26) & 0x7);

    if (PF < 240) {
      /* PDU1 format, the PS contains the destination address */
        dst = PS;
        pgn = (((unsigned long)DP) << 16) | (((unsigned long)PF) << 8);
    } else {
      /* PDU2 format, the destination is implied global and the PGN is extended */
        dst = 0xff;
        pgn = (((unsigned long)DP) << 16) | (((unsigned long)PF) << 8) | (unsigned long)PS;
    }
}

int main()
{
    unsigned char prio, src, dst;
    unsigned long id, pgn;
    std::string name;
    std::cout << "CAN ID? ";
    std::cin >> std::hex >> id;
    std::cout << "CAN ID: " << std::hex << id << "!\n";
    CanIdToN2k(id, prio,pgn,src,dst);
    std::cout << "PGN: " << std::dec << +pgn << "!\n";
    std::cout << "prio: " << +prio<< "!\n";
    std::cout << "src: " << +src << "!\n";
    std::cout << "dst: " << +dst << "!\n";
    
    
}
