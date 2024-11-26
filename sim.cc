#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "sim_proc.h"
#include <vector>
#include <algorithm> // sorting vecotr for age in issue stage

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

// FE to DI - in program order, later out of order
// From DI find an empty spot then transfer

using namespace std;

struct pipeline_entries{
    // for tracking
    // in each state increment the states cycle by 1, we can use age or instruction count?
    // first trying with instruction count
    int instruction_number = 0;
    int cycles_count = 0;
    int op_type = 0;
    int source1=0,source2=0;
    int age = 0;
    bool valid = false;
    int destination=0;
    int fetch=0,decode=0,rename=0,reg_read=0,dispatch=0,issue=0,execute=0,writeback=0,retire=0;
};

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
    uint32_t pc = 0;
};

// struct IQ{  //issue queue
//     bool valid = 0;
//     int destination_tag = 0;
//     bool source1_ready = false;
//     int source1_value = 0;
//     uint32_t source1_tag=0;
//     bool source1_in_rob=false;
//     bool source2_ready=false;
//     int source2_value=0;
//     uint32_t source2_tag=0;
//     bool source2_in_rob=false;
// };

// fetch function not needed

struct DE{
    bool valid = false; // if instructions present true
    int op_type=0;
    int destination = 0;
    int source1 = 0;
    int source1_rob = 0;
    int source2 = 0;
    int source2_rob = 0;
    bool source1_ready=false;
    bool source2_ready=false;
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
    bool source1_ready=false;
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
    bool source1_ready=false;
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
    bool source1_ready=false;
    bool source2_ready=false;
    int source2=0;
    uint32_t source2_tag=0;
    bool source2_in_rob=false;

    int age = 0;
};

struct IQ{
    bool valid = false;
    int op_type = 0;
    int destination = 0;
    uint32_t destination_tag = 0;
    int source1 = 0;
    uint32_t source1_tag=0;
    bool source1_in_rob=false;
    bool source1_ready=false;
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
    bool source1_ready=false;
    bool source2_ready=false;
    int source2=0;
    uint32_t source2_tag=0;
    bool source2_in_rob=false;
    uint32_t timer = 0;
    // bool finished = false;

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
    bool source1_ready=false;
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
    bool source1_ready=false;
    bool source2_ready=false;
    int source2=0;
    uint32_t source2_tag=0;
    bool source2_in_rob=false;

    int age = 0;
};

class superscalar{
    public:
    int head=0,tail=0; // increment whenever
    int width=0,iq_size=0,rob_size=0,ex_width=0, wb_width = 0;
    uint64_t pc = 0;
    int reduce_latency = -1;
    int cycles = 0; // increases in advance cycle only
    // One way to annotate the age of an instruction is to assign an 
    // incrementing sequence number to each instruction as it is fetched from the trace file. instructions_count variable is there to track
    int instructions_count=0;
    vector <pipeline_entries> pseudo_pipeline;
    int OP_LATENCY [3] = {1, 2, 5};

    vector <ROB> rob;
    vector <RMT> rmt;
    vector <RT> retire;
    vector <WB> writeback;
    vector <EX> execute_list;
    vector <DI> dispatch; //dount weather to check readiness
    vector <IQ> issue_q;
    vector <RR> reg_read;
    vector <RN> rename;
    vector <DE> decode;
    int head = 0,tail = 0;

    superscalar(int width,int iq_size, int rob_size){
        this->width = width;
        this->iq_size = iq_size;
        this->rob_size = rob_size;
        this->ex_width = width * 5;
        this->wb_width = width * 5;
        pseudo_pipeline.resize(100000);
        // head_ptr = &rob[0];
        // tail_ptr = &rob[0];
        rob.resize(rob_size);
        writeback.resize(wb_width);
        execute_list.resize(ex_width);
        dispatch.resize(width);
        issue_q.resize(iq_size);
        reg_read.resize(width);
        rename.resize(width);
        decode.resize(width);

        for(int i = 0;i<rob_size;i++){
            rob[i].number = i; //initializing rob0,rob1, indexes
        }
    }

    void Execute(){
        //first find which instruction is finishing executing this cycle
        // checking if latency = 0
        for(int i = 0;i<ex_width;i++){
            if(execute_list[i].valid){
                bool finished_instruction = (execute_list[i].timer == 1);
                execute_list[i].timer = execute_list[i].timer + reduce_latency;
                if(finished_instruction){
                    // 1) Remove the instruction from the execute_list -> just invalid it
                    execute_list[i].valid = false; // removed 
                    // 2) Add the instruction to WB. Find an empty spot
                    for(int j = 0;j<wb_width;j++){
                        if(!writeback[j].valid){
                            //add here
                            writeback[j].valid = true;
                            writeback[j].age = instructions_count; // not sure this time due to timer, maybe execut.age
                            writeback[j].destination = execute_list[i].destination;
                            writeback[j].destination_tag = execute_list[i].destination_tag;
                            writeback[j].op_type = execute_list[i].op_type;
                            writeback[j].source1 = execute_list[i].source1;
                            writeback[j].source1_in_rob = execute_list[i].source1_in_rob;
                            writeback[j].source1_ready = execute_list[i].source1_ready;
                            writeback[j].source1_tag = execute_list[i].source1_tag;

                            writeback[j].source2 = execute_list[i].source2;
                            writeback[j].source2_in_rob = execute_list[i].source2_in_rob;
                            writeback[j].source2_ready = execute_list[i].source2_ready;
                            writeback[j].source2_tag = execute_list[i].source2_tag;
                            pseudo_pipeline[instructions_count].writeback = cycles + 1;
                            break;
                        }
                    }
                }
            }
        }

            // Wakeup dependent instructions (set their source operand ready flags) in 
            // the IQ, DI (the dispatch bundle), and RR (the register-read bundle)
            // compare this->destination with that_stage->source1/2 if not ready make it ready

            //first for IQ source 1
            for(int i = 0;i<ex_width;i++){
                for(int j = 0;j<iq_size;j++){
                    if(execute_list[i].destination == issue_q[j].source1){ // dependent
                        if(!issue_q[j].source1_ready){ //saves time because if true then take time
                            issue_q[j].source1_ready = true; // waked source 1 yay
                        }
                    }
                }
            }
            //for IQ source 2
            for(int i = 0;i<ex_width;i++){
                for(int j = 0;j<iq_size;j++){
                    if(execute_list[i].destination == issue_q[j].source2){ // dependent
                        if(!issue_q[j].source2_ready){ //saves time because if true then take time
                            issue_q[j].source2_ready = true; // waked source 2
                        }
                    }
                }
            } //iq waking out done

            // second wakeup for DI
            for(int i = 0;i<ex_width;i++){
                for(int j = 0;j<width;j++){
                    if(execute_list[i].destination == dispatch[j].source1){ // dependent
                        if(!dispatch[j].source1_ready){ //saves time because if true then take time
                            dispatch[j].source1_ready = true; // waked source 1 yay
                        }
                    }
                }
            }
            //for DI source 2
            for(int i = 0;i<ex_width;i++){
                for(int j = 0;j<width;j++){
                    if(execute_list[i].destination == dispatch[j].source2){ // dependent
                        if(!dispatch[j].source2_ready){ //saves time because if true then take time
                            dispatch[j].source2_ready = true; // waked source 2
                        }
                    }
                }
            } //di waking out done

            // third wakeup for RR
            for(int i = 0;i<ex_width;i++){
                for(int j = 0;j<width;j++){
                    if(execute_list[i].destination == reg_read[j].source1){ // dependent
                        if(!reg_read[j].source1_ready){ //saves time because if true then take time
                            reg_read[j].source1_ready = true; // waked source 1 yay
                        }
                    }
                }
            }
            //for rr source 2
            for(int i = 0;i<ex_width;i++){
                for(int j = 0;j<width;j++){
                    if(execute_list[i].destination == reg_read[j].source2){ // dependent
                        if(!reg_read[j].source2_ready){ //saves time because if true then take time
                            reg_read[j].source2_ready = true; // waked source 2
                        }
                    }
                }
            } //rr waking out done
    }    

    void Issue(){
        int valid_iq_counter = 0; //for tracking instr from IQ till width only  
        int counter2 = 0;
        int min_value = width;// prof - in 1 hidden run the value can be less than width. better then take min of width and that counter
        vector <int> vec; //for oldest to width number - age
        // Issue up to WIDTH oldest instructions from the IQ
        // head, keep on incrementing head: oldest - in rob
        //create an array of that size(number of instructions that are valid and are ready for execution in IQ bundle)
        //store all the valid ready elements(age parameter) in the array

        for(int i = 0; i < width;i++ ){
            if(issue_q[i].valid){
                counter2++;
            }
        }

        for(int i = 0;i<iq_size; i++){
            if(issue_q[i].valid){
                issue_q[i].source1_ready = true; // even if no source make it ready?
                issue_q[i].source2_ready = true;
                valid_iq_counter++;
                vec.push_back(issue_q[i].age);
            }
        }
        
        min_value = min(min_value,valid_iq_counter);

        sort(vec.begin(), vec.end()); // ascending order of ages, from oldest to latest instruction

        // while(valid_iq_counter < width || valid_iq_counter >= 0){ // move until that count
            //1) Remove the instruction from the IQ.
            int temp_index = 0;
            //Add the instruction to the execute_list. and set timer based on op type
            for(int i = 0;i<min_value;i++){
                for(int j = 0; j<iq_size;j++){
                    if(issue_q[j].valid){
                        // valid lets go BRUH
                        if(issue_q[j].age == vec[i]){
                            // checking if the ages same then we start transfering
                            issue_q[j].valid = false;
                            temp_index = j;
                            break;
                        }
                    }
                }
                // now put in ex
                for(int k = 0;k<ex_width;k++){
                    //finding empty place since out of order
                    if(!execute_list[k].valid){// found empty
                        // removing the instruction from IQ
                        issue_q[temp_index].valid = false;  // see nov 18 class at 39:30 then why valid = 0.
                        // after it goes to next stage it wont be in IQ
                        // adding the instruction to ex
                        execute_list[k].valid = true;
                        execute_list[k].age = instructions_count;
                        execute_list[k].destination = issue_q[temp_index].destination;
                        execute_list[k].destination_tag = issue_q[temp_index].destination_tag;
                        execute_list[k].op_type = issue_q[temp_index].op_type;
                        execute_list[k].source1 = issue_q[temp_index].source1;
                        execute_list[k].source1_in_rob = issue_q[temp_index].source1_in_rob;
                        execute_list[k].source1_ready = issue_q[temp_index].source1_ready;
                        execute_list[k].source1_tag = issue_q[temp_index].source1_tag;
                        execute_list[k].source2 = issue_q[temp_index].source2;
                        execute_list[k].source2_in_rob = issue_q[temp_index].source2_in_rob;
                        execute_list[k].source2_ready = issue_q[temp_index].source2_ready;
                        execute_list[k].source2_tag = issue_q[temp_index].source2_tag;
                        execute_list[k].timer = OP_LATENCY[execute_list[k].op_type]; //  will allow you to model its execution latency.
                        pseudo_pipeline[instructions_count].execute = cycles + 1;
                        break; // now searching for next free space
                        }
                    }
                }
            // valid_iq_counter = valid_iq_counter - 1;
        // }
    }
    
    void Dispatch(){
        int invalid_value = -1;
        bool bundle_present = false;
        for(int i = 0;i<width;i++){
            if(dispatch[i].valid){
                bundle_present = true;
                break;
            }
        }
        if(bundle_present){
            int free_entries = 0;
            bool enough_entries = false;
            //  If the number of free IQ entries is less do nothing
            for(int i = 0;i<iq_size;i++){
                if(!issue_q[i].valid){
                    free_entries += 1;
                    if(free_entries >= width){
                        enough_entries = true;
                        break;
                    }
                }
            }


            // Readiness of each source register must be individually ascertained.  That should be the context for interpreting my previous reply.
            // An instruction's overall readiness is evaluated by considering readiness of all its source registers.
            
            //if valid instruction
            // order is rmt s1->rob s1-tag->make it ready bruh
            // check if source valid in rmt
            // if yes then using rob tag check if ready in rob
            for(int i=0;i<width;i++){
                if(dispatch[i].valid){
                    bool double_check = dispatch[i].source1_in_rob;
                    if(rmt[dispatch[i].source1].valid){
                        // check in rob
                        // if valid then make source ready only if rob ready and in rob (i.e thought didnt execute yet) -> lets go true it
                        // if(dispatch[i].source1_in_rob){ // hardcode to true?
                        if(rob[dispatch[i].source1_tag].ready && double_check){
                            dispatch[i].source1_ready = true;    
                        }
                    }
                    else if(!rmt[dispatch[i].source1].valid){
                        // then no source present so not waiting to execute make it true anyway
                        dispatch[i].source1_ready = true;  
                    }
                }
            }

            if(enough_entries){
                // dispatch all instructions from DI to the IQ
                for(int i = 0;i<width;i++){
                    if(dispatch[i].valid){
                        for(int j = 0;j<iq_size;j++){
                            if(!issue_q[j].valid){
                                // empty spot found move all from DI
                                issue_q[j].valid = true;//dispatch[i].valid;
                                issue_q[j].op_type = dispatch[i].op_type;
                                issue_q[j].age   = dispatch[i].age; //is instruction count == to this
                                issue_q[j].destination = dispatch[i].destination;
                                issue_q[j].destination_tag = dispatch[i].destination_tag;
                                issue_q[j].source1 = dispatch[i].source1;
                                issue_q[j].source1_in_rob = dispatch[i].source1_in_rob;
                                issue_q[j].source1_tag = dispatch[i].source1_tag;
                                issue_q[j].source1_ready = dispatch[i].source1_ready;
                                issue_q[j].source2 = dispatch[i].source2;
                                issue_q[j].source2_in_rob = dispatch[i].source2_in_rob;
                                issue_q[j].source2_tag = dispatch[i].source2_tag;
                                issue_q[j].source2_ready = dispatch[i].source2_ready;
                                dispatch[i].valid = false; // removed instruction
                                pseudo_pipeline[instructions_count].issue = cycles + 1;
                                break; // breaks out of for j loop since its valid now looking for another spot
                            }
                        }
                    }
                }
            }
        }

    }

    void RegRead(){
        // If RR contains a register-read bundle:
        bool bundle_present = false;
        bool dispatch_empty = true;
        int invalid_value = -1;
        
        for(int i = 0;i<width;i++){
            if(reg_read[i].valid){
                bundle_present = true;
                break;
            }
        }

        for(int i =0;i<width;i++){
            if(dispatch[i].valid){
                dispatch_empty = false;
                break;
                return;
            }
        }

        /*
         If DI is empty and bundle present
        // Since values are not explicitly modeled, the sole purpose of the Register Read 
        // stage is to ascertain the readiness of the renamed source operands. Always putting sourcei.ready = true
        */

       if(bundle_present && dispatch_empty){
            for(int i = 0;i<width;i++){
                if(reg_read[i].valid){
/*
- For any valid source register, readiness as evaluated in RegRead() depends on two factors: 
(1) which register file partition it was renamed to in Rename() (either the ARF -- whose committed values are always ready -- or the ROB) and
(2) if it was renamed to the ROB (linked to a producer in the ROB), the producer's ROB entry has a ready bit that is managed by the producer (it may or may not be ready yet, depending on if and when the producer completed).
  This evaluation of readiness based on the above two factors was exhibited in various scenarios in the detailed simulation from class.
*/
                    // check if valid source | if not ready then set it ready                 
                    int temp1 = reg_read[i].source1;
                    if(temp1 != invalid_value){
                        //check rmt for this source if valid then it is not executed yet
                        // then also set ready
                        if(rmt[temp1].valid){
                            // check if destination is the source; and valid in rob and ready
                            if(rob[reg_read[i].source1_tag].valid && rob[reg_read[i].source1_tag].destination == reg_read[i].source1){
                                if(rob[reg_read[i].source1_tag].ready){
                                    reg_read[i].source1_ready = true;
                                } // by default I have put false so no else needed
                            }
                        }
                        else{ // not in rmt then in arf so ready
                            reg_read[i].source1_ready = true;
                        }
                        // set ready anyway. should we also set ready if no source? ask Prof: said yes or omit it
                    }
                    else if(temp1 == invalid_value){ // make it truw so that it doesnt cause delay
                        reg_read[i].source1_ready = true;
                    }

                    // same for s2
                    int temp2 = reg_read[i].source2;
                    if(temp2 != invalid_value){
                        //check rmt for this source if valid then it is not executed yet
                        // then also set ready
                        if(rmt[temp2].valid){
                            if(rob[reg_read[i].source2_tag].valid && rob[reg_read[i].source2_tag].destination == reg_read[i].source2){
                                if(rob[reg_read[i].source2_tag].ready){
                                    reg_read[i].source2_ready = true;
                                } // by default i put false so no else needed
                            }
                        }
                        else{
                            reg_read[i].source2_ready = true;
                        }
                        // set ready anyway. should we also set ready if no source? ask Prof
                    }
                    else if(temp2 == invalid_value){
                        reg_read[i].source2_ready = true; //set anyway to avoid delay
                    }
                    // from next stage onwards goes OOO so always look for empty place
                    // process (see below) the register-read bundle and advance it from RR to DI. we look for empty place in di
                    bool empty_spot = !dispatch[i].valid;
                    if(empty_spot){
                        dispatch[i].age = instructions_count;
                        dispatch[i].valid = reg_read[i].valid; // true
                        dispatch[i].op_type = reg_read[i].op_type;
                        dispatch[i].source1 = reg_read[i].source1;
                        dispatch[i].source1_ready = reg_read[i].source1_ready;
                        dispatch[i].source1_tag = reg_read[i].source1_tag;
                        dispatch[i].source1_in_rob = reg_read[i].source1_in_rob;
                        dispatch[i].source2 = reg_read[i].source2;
                        dispatch[i].source2_ready = reg_read[i].source2_ready;
                        dispatch[i].source2_tag = reg_read[i].source2_tag;
                        dispatch[i].source2_in_rob = reg_read[i].source2_in_rob;
                        reg_read[i].valid = false; // removed instruction
                        pseudo_pipeline[instructions_count].dispatch = cycles + 1;
                    }
                }
            }
       }

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
                //Allocate entry, increment the tail and inherit the tag from rob
                for(int i = 0;i<width;i++){
                    if(rename[i].valid){ // only change for valid instr
                        // now check source, destination there or not
                        if(rename[i].source1 != invalid_value){
                            // valid source reg
                            int temp = rename[i].source1;
                            if(rmt[temp].valid){
                                // then it is waiting for value not executed
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
                                // if there in RMT then it is waiting for value in pipeline, not commited version
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
                            rmt[temp_destination].tag = rob[tail].number; //rob{tail number}
                        } // if no destination then do nothing

                        // need to increment tail so that next instruction can use it, if points to last then make it 0
                        tail = (tail == 66)?(0):(tail + 1); // since circular if end we start from 0

                        //and advance it from RN TO RR
                        reg_read[i].valid = rename[i].valid;
                        reg_read[i].destination = rename[i].destination;
                        reg_read[i].destination_tag = rename[i].destination_tag;
                        reg_read[i].op_type = rename[i].op_type;
                        reg_read[i].source1 = rename[i].source1;
                        reg_read[i].source1_in_rob = rename[i].source1_in_rob;
                        reg_read[i].source1_tag = rename[i].source1_tag;
                        reg_read[i].source1_ready = rename[i].source1_ready;
                        reg_read[i].source2 = rename[i].source2;
                        reg_read[i].source2_in_rob = rename[i].source2_in_rob;
                        reg_read[i].source2_ready = rename[i].source2_ready;
                        reg_read[i].source2_tag = rename[i].source2_tag;
                        rename[i].valid = false; // removed instruction
                        pseudo_pipeline[instructions_count].reg_read = cycles + 1;
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
                        rename[i].source1_ready = decode[i].source1_ready; // default false
                        rename[i].source2_ready = decode[i].source2_ready;
                        decode[i].valid = false;
                        // rename age leda inst cont
                        pseudo_pipeline[instructions_count].rename = cycles + 1;
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
                    instructions_count += 1; // increase after each read from file
                    decode[i].valid = true;
                    decode[i].op_type = op_type;
                    decode[i].destination = destination;
                    decode[i].source1 = source1;
                    decode[i].source2 = source2;
                    decode[i].age=instructions_count;

                    //now putting into serial pipeline
                    pseudo_pipeline[instructions_count].instruction_number = instructions_count;
                    pseudo_pipeline[instructions_count].cycles_count = cycles;
                    pseudo_pipeline[instructions_count].op_type = op_type;
                    pseudo_pipeline[instructions_count].source1 = source1;
                    pseudo_pipeline[instructions_count].source2 = source2;
                    pseudo_pipeline[instructions_count].destination = destination;
                    pseudo_pipeline[instructions_count].valid = true;
                    pseudo_pipeline[instructions_count].fetch = cycles;
                    pseudo_pipeline[instructions_count].decode = cycles + 1;


                    if(feof(fp)){
                        break; // no more instructions
                    }
                }
            }
        }
    }

    void Advance_Cycle(){

    }
    
};

// One way to annotate the age of an instruction is to assign an 
// incrementing sequence number to each instruction as it is fetched from the trace file. Instruction_count variable is there to track

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
