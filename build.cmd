xc16-gcc -O3 FMOgre_DMA_Main.c -o fmogre.elf -mcpu=33FJ128GP804  -T p33FJ128GP804.gld
xc16-bin2hex fmogre.elf
