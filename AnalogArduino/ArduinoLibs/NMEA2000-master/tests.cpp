#include <stdio.h>
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
	unsigned char prio1, src1, dst1, prio2, src2, dst2, prio3, src3, dst3, prio4, src4, dst4;
	unsigned long pgn1, pgn2, pgn3, pgn4;
	unsigned long id1 = 0x11F80F2B;
	unsigned long id2 = 0x11F80E2B;
	unsigned long id3 = 0x9F8012B;
	unsigned long id4 = 0x9F8022B;
	CanIdToN2k(id1, prio1, pgn1, src1, dst1);
	CanIdToN2k(id2, prio2, pgn2, src2, dst2);
	CanIdToN2k(id3, prio3, pgn3, src3, dst3);
	CanIdToN2k(id4, prio4, pgn4, src4, dst4);
	printf("Prio: %i, PGN: %i, SRC: %i, DST: %i\n",prio1, pgn1, src1, dst1);
	printf("Prio: %i, PGN: %i, SRC: %i, DST: %i\n",prio2, pgn2, src2, dst2);
	printf("Prio: %i, PGN: %i, SRC: %i, DST: %i\n",prio3, pgn3, src3, dst3);
	printf("Prio: %i, PGN: %i, SRC: %i, DST: %i\n",prio4, pgn4, src4, dst4);
}