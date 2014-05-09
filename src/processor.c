/* processor.c --- processor simulation for the 16candles VM
   Copyright (c) 2014 Jack Pugmire, Joe Jevnik, and Ted Meyer

   This program is free software; you can redistribute it and/or modify it
   under the terms of the GNU General Public License as published by the Free
   Software Foundation; either version 2 of the License, or (at your option)
   any later version.

   This program is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
   more details.

   You should have received a copy of the GNU General Public License along with
   this program; if not, write to the Free Software Foundation, Inc., 51
   Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA. */

#include "../16machine/machine/processor.h"

// Initializes all registers and subregisters.
void init_regs(){
    c16_halfword *rs = calloc(32,sizeof(c16_halfword));
    ipt   = (c16_reg) &rs[0];
    spt   = (c16_reg) &rs[2];
    ac1   = (c16_reg) &rs[4];
    ac2   = (c16_reg) &rs[6];
    tst   = (c16_reg) &rs[8];
    inp   = (c16_reg) &rs[10];
    inp_r = &rs[10];
    inp_w = &rs[11];
    r0    = (c16_reg) &rs[12];
    r0_f  = &rs[13];
    r0_b  = &rs[12];
    r1    = (c16_reg) &rs[14];
    r1_f  = &rs[15];
    r1_b  = &rs[14];
    r2    = (c16_reg) &rs[16];
    r2_f  = &rs[17];
    r2_b  = &rs[16];
    r3    = (c16_reg) &rs[18];
    r3_f  = &rs[19];
    r3_b  = &rs[20];
    r4    = (c16_reg) &rs[20];
    r4_f  = &rs[21];
    r4_b  = &rs[29];
    r5    = (c16_reg) &rs[22];
    r5_f  = &rs[23];
    r5_b  = &rs[22];
    r6    = (c16_reg) &rs[24];
    r6_f  = &rs[25];
    r6_b  = &rs[24];
    r7    = (c16_reg) &rs[26];
    r7_f  = &rs[27];
    r7_b  = &rs[26];
    r8    = (c16_reg) &rs[28];
    r8_f  = &rs[29];
    r8_b  = &rs[28];
    r9    = (c16_reg) &rs[30];
    r9_f  = &rs[32];
    r9_b  = &rs[31];
}

void free_regs(){
    free(ipt);
}

// Fills the register with the next word at the ipt.
// WARNING: Do not call on ipt, intermediate reading will corrupt the value.
void fill_word(c16_reg reg){
    *reg =  (c16_word) sysmem.mem[(*ipt)++] << 8;
    *reg += sysmem.mem[(*ipt)++];
}

// simulate one processor tick, returning -1 if an exit opcode was encountered
int proc_tick(){
    c16_opcode op = sysmem.mem[(*ipt)++];
    if (op == OP_TERM){ // exit case
        return -1;
    }
    if (op <= OP_MAX_REG_REG){             // binary operators
        op_bin_ops(op);
    }else if (op <= OP_LT_REG_REG){        // comparison operators
        op_cmp_ops(op);
    }else if (op <= OP_SET_REG){           // unary operators
        op_un_ops(op);
    }else if ((op >> 1) << 1 == OP_PUSH_){ // op push
        op_push(op);
    }else if (op <= OP_JMPF){              // jump operations
        op_jmp(op);
    }else if (op <= OP_WRITE_REG){         // escaping write
        op_write(op);
    }else if (op <= OP_MSET_MEMREG){       // memset operations
        op_mset(op);
    }else if (op == OP_SWAP){              // swap operator
        op_swap();
    }else if (op == OP_POP){               // pop operator
        op_pop();
    }else if (op == OP_PEEK){              // peek operator
        op_peek();
    }else if (op == OP_FLUSH){             // flush the stack operator
        op_flush();
    }else if (op == OP_READ){              // escaping read
        op_read();
    }

    return 0;
}

// Subtracts two timeval structures storing the result in the result struct.
// Returns 1 if result is negative.
// Source: http://www.gnu.org/software/libc/manual/html_node/Elapsed-Time.html
int timeval_subtract (struct timeval *result,struct timeval *x,
                      struct timeval *y){
    int nsec;
    if (x->tv_usec < y->tv_usec){
        nsec = (y->tv_usec - x->tv_usec) / 1000000 + 1;
        y->tv_usec -= 1000000 * nsec;
        y->tv_sec += nsec;
    }
    if (x->tv_usec - y->tv_usec > 1000000){
        nsec = (x->tv_usec - y->tv_usec) / 1000000;
        y->tv_usec += 1000000 * nsec;
        y->tv_sec -= nsec;
    }
    result->tv_sec = x->tv_sec - y->tv_sec;
    result->tv_usec = x->tv_usec - y->tv_usec;
    return x->tv_sec < y->tv_sec;
}

// Run the processor until termination, executing the current program in RAM.
// The parameter is the minimum length in micro seconds of a single tick.
// If tick_len is <= 0, no time checking shall occur (program runs with
// unbounded speed and ticks will have variable length).
void run(long tick_len){
    struct timeval d,b,a; // Difference, Before, After.
    int pt;
    if (tick_len <= 0){
        while(proc_tick() != -1);
    }else{
        do{
            gettimeofday(&b,NULL);
            pt = proc_tick();
            for (gettimeofday(&a,NULL),timeval_subtract(&d,&a,&b);
                 d.tv_usec < tick_len;
                 gettimeofday(&a,NULL),timeval_subtract(&d,&a,&b));
        }while(pt != -1);
    }
}

// Returns the register from the given byte.
// return: The reg or subreg that the opcode describes.
void *parse_reg(c16_halfword reg){
    switch(reg){
    case OP_ipt:   return ipt;
    case OP_spt:   return spt;
    case OP_ac1:   return ac1;
    case OP_ac2:   return ac2;
    case OP_tst:   return tst;
    case OP_inp:   return inp;
    case OP_inp_r: return inp_r;
    case OP_inp_w: return inp_w;
    case OP_r0:    return r0;
    case OP_r0_f:  return r0_f;
    case OP_r0_b:  return r0_b;
    case OP_r1:    return r1;
    case OP_r1_f:  return r1_f;
    case OP_r1_b:  return r1_b;
    case OP_r2:    return r2;
    case OP_r2_f:  return r2_f;
    case OP_r2_b:  return r2_b;
    case OP_r3:    return r3;
    case OP_r3_f:  return r3_f;
    case OP_r3_b:  return r3_b;
    case OP_r4:    return r4;
    case OP_r4_f:  return r4_f;
    case OP_r4_b:  return r4_b;
    case OP_r5:    return r5;
    case OP_r5_f:  return r5_f;
    case OP_r5_b:  return r5_b;
    case OP_r6:    return r6;
    case OP_r6_f:  return r6_f;
    case OP_r6_b:  return r6_b;
    case OP_r7:    return r7;
    case OP_r7_f:  return r7_f;
    case OP_r7_b:  return r7_b;
    case OP_r8:    return r8;
    case OP_r8_f:  return r8_f;
    case OP_r8_b:  return r8_b;
    case OP_r9:    return r9;
    case OP_r9_f:  return r9_f;
    case OP_r9_b:  return r9_b;
    default:       return NULL;
    }
}

// Prints the state of the machines registers to stdout.
void dump_regs(){
    printf(
        "ipt: 0x%04x\n"
        "spt: 0x%04x\n"
        "ac1: 0x%04x\n"
        "ac2: 0x%04x\n"
        "tst: 0x%04x\n"
        "inp: 0x%04x: inp_w: 0x%02x, inp_r: 0x%02x\n"
        "r0:  0x%04x: r0_f:  0x%02x, r0_b:  0x%02x\n"
        "r1:  0x%04x: r1_f:  0x%02x, r1_b:  0x%02x\n"
        "r2:  0x%04x: r2_f:  0x%02x, r2_b:  0x%02x\n"
        "r3:  0x%04x: r3_f:  0x%02x, r3_b:  0x%02x\n"
        "r4:  0x%04x: r4_f:  0x%02x, r4_b:  0x%02x\n"
        "r5:  0x%04x: r5_f:  0x%02x, r5_b:  0x%02x\n"
        "r6:  0x%04x: r6_f:  0x%02x, r6_b:  0x%02x\n"
        "r7:  0x%04x: r7_f:  0x%02x, r7_b:  0x%02x\n"
        "r8:  0x%04x: r8_f:  0x%02x, r8_b:  0x%02x\n"
        "r9:  0x%04x: r9_f:  0x%02x, r9_b:  0x%02x\n",
        *ipt,*spt,*ac1,*ac2,*tst,*inp,*inp_w,*inp_r,
        *r0,*r0_f,*r0_b,*r1,*r1_f,*r1_b,*r2,*r2_f,*r2_b,*r3,*r3_f,*r3_b,
        *r4,*r4_f,*r4_b,*r5,*r5_f,*r5_b,*r6,*r6_f,*r6_b,*r7,*r7_f,*r7_b,
        *r8,*r8_f,*r8_b,*r9,*r9_f,*r9_b);
}

// Process the stdin in a second thread.
// pass a NULL, it isn't used.
void *process_stdin(void *_){
    int c;
    while ((c = getchar()) != EOF){
        sysmem.inputv[(*inp_w)++] = (c16_halfword) c;
        if (*sysmem.inputc < 256){
            ++(*sysmem.inputc);
        }
    }
    *sysmem.inputb = 0;
    return NULL;
}

int main(int argc,char **argv){
    FILE *in;
    int n,cs = 0;
    pthread_t input_thread;
    init_regs();
    init_mem_default(&sysmem);
    if (argc == 1){
        puts("Usage: 16c BINARY");
        return 0;
    }
    in = fopen(argv[1],"r");
    if (!in){
        fprintf(stderr,"Error: Unable to open file '%s'\n");
        return -1;
    }
    load_file(&sysmem,0,in);
    pthread_create(&input_thread,NULL,process_stdin,NULL);
    run(50);
    // dump_regs(); // dump the values of all the registers when a program stops
    pthread_cancel(input_thread);
    free_regs();
    free_mem(&sysmem);
    return EXIT_SUCCESS;
}
