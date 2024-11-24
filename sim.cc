#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "sim_proc.h"
#include <vector>

/*  argc holds the number of command line arguments
    argv[] holds the commands themselves

    Example:-
    sim 256 32 4 gcc_trace.txt
    argc = 5
    argv[0] = "sim"
    argv[1] = "256"
    argv[2] = "32"
    ... and so on
*/

using namespace std;

struct RMT{  //rename map table
    bool valid = 0;
    uint32_t tag=0;
};

struct ROB{
    int number = 0; // rob0,rob1 ...
    int age; // do we need valid bit
    int destination = 0;
    // no mis,exe needed
    bool ready = false;
    bool valid = false;
    struct ROB *head;  // Head pointer
    struct ROB *tail;  // Tail pointer
    uint32_t pc = 0;
};

struct IQ{  //issue queue
    bool valid = 0;
    int destination_tag = 0;
    bool source1_ready = false;
    int source1_value = 0;
    uint32_t source1_tag=0;
    bool source1_in_rob=false;
    bool source2_ready=false;
    int source2_value=0;
    uint32_t source2_tag=0;
    bool source2_in_rob=false;
};

// fetch function not needed

struct DE{
    bool valid = false; // if instructions present true
    int op_type=0;
    int destination = 0;
    int source1 = 0;
    int source1_rob = 0;
    int source2 = 0;
    int source2_rob = 0;

    int age = 0;
};

// from RN stages sourcei_in_rob to tell if srci will be produced by an instruction currently in the pipeline. The instruction must wait for this value before proceeding.

struct RN{
    bool valid = false;
    int op_type = 0;
    int destination = 0;
    uint32_t destination_tag = 0;
    int source1 = 0;
    uint32_t source1_tag=0;
    bool source1_in_rob=false;
    bool source2_ready=false;
    int source2=0;
    uint32_t source2_tag=0;
    bool source2_in_rob=false;

    int age = 0;
};

struct RR{
    bool valid = false;
    int op_type = 0;
    int destination = 0;
    uint32_t destination_tag = 0;
    int source1 = 0;
    uint32_t source1_tag=0;
    bool source1_in_rob=false;
    bool source2_ready=false;
    int source2=0;
    uint32_t source2_tag=0;
    bool source2_in_rob=false;

    int age = 0;
};

struct DI{
    bool valid = false;
    int op_type = 0;
    int destination = 0;
    uint32_t destination_tag = 0;
    int source1 = 0;
    uint32_t source1_tag=0;
    bool source1_in_rob=false;
    bool source2_ready=false;
    int source2=0;
    uint32_t source2_tag=0;
    bool source2_in_rob=false;

    int age = 0;
};

struct IS{
    bool valid = false;
    int op_type = 0;
    int destination = 0;
    uint32_t destination_tag = 0;
    int source1 = 0;
    uint32_t source1_tag=0;
    bool source1_in_rob=false;
    bool source2_ready=false;
    int source2=0;
    uint32_t source2_tag=0;
    bool source2_in_rob=false;

    int age = 0;
};

struct EX{
    bool valid = false;
    int op_type = 0;
    int destination = 0;
    uint32_t destination_tag = 0;
    int source1 = 0;
    uint32_t source1_tag=0;
    bool source1_in_rob=false;
    bool source2_ready=false;
    int source2=0;
    uint32_t source2_tag=0;
    bool source2_in_rob=false;

    int age = 0;
};

struct WB{
    bool valid = false;
    int op_type = 0;
    int destination = 0;
    uint32_t destination_tag = 0;
    int source1 = 0;
    uint32_t source1_tag=0;
    bool source1_in_rob=false;
    bool source2_ready=false;
    int source2=0;
    uint32_t source2_tag=0;
    bool source2_in_rob=false;

    int age = 0;
};

struct RT{
    bool valid = false;
    int op_type = 0;
    int destination = 0;
    uint32_t destination_tag = 0;
    int source1 = 0;
    uint32_t source1_tag=0;
    bool source1_in_rob=false;
    bool source2_ready=false;
    int source2=0;
    uint32_t source2_tag=0;
    bool source2_in_rob=false;

    int age = 0;
};

class superscalar{
    public:
    int head=0,tail=0; // increment whenever
    int width=0,iq_size=0,rob_size=0;
    uint64_t pc = 0;
    int instructions_count=0;
    struct ROB *head_ptr; // pointer to the head of rob
    struct ROB *tail_ptr; // tail of rob, can;t put in struct ig
    vector <ROB> rob;
    vector <RMT> rmt;
    vector <RT> retire;
    vector <WB> writeback;
    vector <EX> execute_list;
    vector <DI> dispatch;
    vector <IS> issue_q;
    vector <RR> reg_read;
    vector <RN> rename;
    vector <DE> decode;

    superscalar(int width,int iq_size, int rob_size){
        this->width = width;
        this->iq_size = iq_size;
        this->rob_size = rob_size;
        head_ptr = &rob[0];
        tail_ptr = &rob[0];
        rob.resize(rob_size);
        writeback.resize(width*5);
        execute_list.resize(width*5);
        dispatch.resize(width);
        issue_q.resize(iq_size);
        reg_read.resize(width);
        rename.resize(width);
        decode.resize(width);
    }
    
    void Rename(){
        // If RN contains a rename bundle: 
        bool bundle_present = false;
        int invalid_value = -1; // if equal to this do nothing

        for(int i = 0;i<width;i++){
            if(rename[i].valid){
                bundle_present = true;
                break;
            }
        }
        //  RR is empty
        bool rr_empty = true;
        for(int i = 0;i<width;i++){
            if(reg_read[i].valid){
                rr_empty = false; //if atleast one not empty do nothing
                return;
            }
        }
        
        if(bundle_present){
            //the ROB has enough free entries >= width
            int rob_counter = 0;
            for(int i = 0;i<67;i++){
                if(!rob[i].valid){
                    rob_counter++;
                }
            }
            bool enough_spaces_in_rob = rob_counter >= width;

            if(enough_spaces_in_rob && rr_empty){
                for(int i = 0;i<width;i++){
                    if(rename[i].valid){ // only change for valid
                        // now check source, destination there or not

                        if(rename[i].source1 != invalid_value){
                            // valid source reg
                            int temp = rename[i].source1;
                            if(rmt[temp].valid){
                                // then it is waiting for value in pipeline
                                rename[i].source1_in_rob = true;
                                rename[i].source1_tag = rmt[temp].tag;
                                // tag given to rename. for mapping
                            }
                            else{
                                // if not waiting then in ARF, set to 0
                                rename[i].source1_in_rob = false;
                                // not there in rob
                            }
                        }
                        // same thing for s2
                        if(rename[i].source2 != invalid_value){
                            // valid source reg
                            int temp = rename[i].source2;
                            if(rmt[temp].valid){
                                // if there in RMT then it is waiting for value in pipeline
                                rename[i].source2_in_rob = true;
                                rename[i].source2_tag = rmt[temp].tag;
                                // tag given to rename. for mapping
                            }
                            else{
                                // if not waiting then in ARF, set to 0
                                rename[i].source2_in_rob = false;
                                // not there in rob
                            }
                        }
                        //  (3) rename its destination register (if it has one)
                        // now ahve to assign dest in rob at tail
                        int temp_destination = rename[i].destination;
                        rob[tail].valid = true;
                        rob[tail].ready = false;
                        rob[tail].destination = temp_destination;
                        // now need to put rob number as tag to rmt
                        if(temp_destination != invalid_value){
                            rmt[temp_destination].valid = true;
                            rmt[temp_destination].tag = rob[tail].number;
                        }
                        // need to increment tail then
                        tail = tail + 1
                    }
                }
            }
        }

    }

    void Decode(){
        bool bundle_present = false;
        bool rename_vacant = true;

        for(int i = 0;i<width;i++){
            if(decode[i].valid){
                bundle_present = true;
                break;
            }
        }

        if(bundle_present){
            for(int i = 0;i < width;i++){
                if(rename[i].valid){
                    rename_vacant = false; // not empty, do nothing
                    return;
                }
            }
            // if total empty should we check if decode i valid also
            if (rename_vacant){
                for(int i=0;i < width;i++){
                    if(decode[i].valid){
                        rename[i].op_type = decode[i].op_type;
                        rename[i].destination = decode[i].destination;
                        rename[i].source1 = decode[i].source1;
                        rename[i].source2 = decode[i].source2;
                        rename[i].age = decode[i].age;
                        rename[i].valid = decode[i].valid;
                        decode[i].valid = false;
                    }                    
                }
            }
            
        }
    }

    void Fetch(FILE *fp){
        bool decode_empty = true;
        int op_type = 0;
        int destination = 0;
        int source1 = 0;
        int source2 = 0;
        uint32_t pc = 0;
        int decode_instructions_count=0;
        for(int i=0;i<width;i++){
            if(decode[i].valid){
                decode_empty = false; //not empty
                return;
            }
        }

        if(!(feof(fp))){  //instructions in trace file 
            if(decode_empty){    //if DE is empty
                for(int i=0;i<width;i++){    //upto width only
                    fscanf(fp,"%llx %d %d %d %d\n",&pc,&op_type,&destination,&source1,&source2);
                    decode[i].valid = true;
                    decode[i].op_type = op_type;
                    decode[i].destination = destination;
                    decode[i].source1 = source1;
                    decode[i].source2 = source2;
                    instructions_count += 1;
                    decode[i].age=instructions_count;


                    if(feof(fp)){
                        break; // no more instructions
                    }
                }
            }
        }
    }
    
};


int main (int argc, char* argv[])
{
    FILE *FP;               // File handler
    char *trace_file;       // Variable that holds trace file name;
    proc_params params;       // look at sim_bp.h header file for the the definition of struct proc_params
    int op_type, dest, src1, src2;  // Variables are read from trace file
    uint64_t pc; // Variable holds the pc read from input file
    
    if (argc != 5)
    {
        printf("Error: Wrong number of inputs:%d\n", argc-1);
        exit(EXIT_FAILURE);
    }
    
    params.rob_size     = strtoul(argv[1], NULL, 10);
    params.iq_size      = strtoul(argv[2], NULL, 10);
    params.width        = strtoul(argv[3], NULL, 10);
    trace_file          = argv[4];
    printf("rob_size:%lu "
            "iq_size:%lu "
            "width:%lu "
            "tracefile:%s\n", params.rob_size, params.iq_size, params.width, trace_file);
    // Open trace_file in read mode
    FP = fopen(trace_file, "r");
    if(FP == NULL)
    {
        // Throw error and exit if fopen() failed
        printf("Error: Unable to open file %s\n", trace_file);
        exit(EXIT_FAILURE);
    }
    // from the fetch function we read the tracefile

    // do{

    // }while(Advance_Cycle());


    //
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    //
    // The following loop just tests reading the trace and echoing it back to the screen.
    //
    // Replace this loop with the "do { } while (Advance_Cycle());" loop indicated in the Project 3 spec.
    // Note: fscanf() calls -- to obtain a fetch bundle worth of instructions from the trace -- should be
    // inside the Fetch() function.
    //
    ///////////////////////////////////////////////////////////////////////////////////////////////////////
    while(fscanf(FP, "%lx %d %d %d %d", &pc, &op_type, &dest, &src1, &src2) != EOF)
        printf("%lx %d %d %d %d\n", pc, op_type, dest, src1, src2); //Print to check if inputs have been read correctly

    return 0;
}
