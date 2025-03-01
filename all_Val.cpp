//------------------------------------------------------------------------------------------------------------
// Filename    : sim_proc.cc
// Author      : Gunavant Setty
// Affiliation : North Carolina State University, Raleigh, NC
// Date        : Dec 2024
// Email       : gsetty@ncsu.edu, gunavantsetty26@gmail.com
// Description : The code successfully realizes the Superscalar Pipeline Simulator,
//	             including all the nine stages for different processor configuration {ROB_SIZE, IQ_SIZE, WIDTH}.
//               The program also outputs the total cycles and the Instructions Per Cycle (IPC)                
//------------------------------------------------------------------------------------------------------------
// Project 3: Superscalar Pipeline Simulator
//------------------------------------------------------------------------------------------------------------

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
    // first trying with present instruction number 
    int instruction_number = 0;
    int cycles_count = 0;
    int op_type = 0;
    int source1=0,source2=0;
    int age = 0;
    bool valid = false;
    int destination=0;
};

struct pipeline_begin_cycles{
    int fetch = 0;
    int decode=0;
    int rename=0;
    int reg_read=0;
    int dispatch=0;
    int issue=0;
    int execute=0;
    int writeback = 0;
    int retire = 0;
    int end = 0;
    bool end_of_instruction = false;
    // this is for knowing the begin-cycle and duration, like which cycle
    //for duration we can subtract by next stage begin-cycle; trying  
    int fetch_duration=0,decode_duration=0,rename_duration=0,reg_read_duration=0,dispatch_duration=0,issue_duration=0,execute_duration=0,writeback_duration=0,retire_duration=0;
};

// fetch function not needed

struct DE{
    bool valid = false; // if instructions present true
    int op_type=0;
    int destination = 0;
    int source1 = 0;
    int source2 = 0;
    bool source1_ready=false;
    bool source2_ready=false;
    int age = 0;
};

// from RN stages sourcei_in_rob to tell if srci will be produced by an instruction currently in the pipeline. The instruction must wait for this value before proceeding.

struct RN{
    bool valid = false;
    int op_type = 0;
    int destination = 0;
    uint64_t destination_tag = 0;
    int source1 = 0;
    uint64_t source1_tag=0;
    bool source1_renamed=false;
    bool source1_ready=false;
    bool source2_ready=false;
    int source2=0;
    uint64_t source2_tag=0;
    bool source2_renamed=false;

    int age = 0;
};

struct RR{
    bool valid = false;
    int op_type = 0;
    int destination = 0;
    uint64_t destination_tag = 0;
    int source1 = 0;
    uint64_t source1_tag=0;
    bool source1_renamed=false;
    bool source1_ready=false;
    bool source2_ready=false;
    int source2=0;
    uint64_t source2_tag=0;
    bool source2_renamed=false;

    int age = 0;
};

struct DI{
    bool valid = false;
    int op_type = 0;
    int destination = 0;
    uint64_t destination_tag = 0;
    int source1 = 0;
    uint64_t source1_tag=0;
    bool source1_renamed=false;
    bool source1_ready=false;
    bool source2_ready=false;
    int source2=0;
    int stall_cycles = 0;
    uint64_t source2_tag=0;
    bool source2_renamed=false;

    int age = 0;
};

struct RMT{  //rename map table
    bool valid = 0;
    uint64_t tag=0;
};

struct ROB{
    uint64_t rob_index = 0; // rob0,rob1 ...
    int age= 0; // do we need valid bit
    int destination = 0;
    // no mis,exe needed
    bool ready = false;
    bool valid = false;
    uint64_t pc = 0;
};

struct IQ{
    bool valid = false;
    int op_type = 0;
    int destination = 0;
    uint64_t destination_tag = 0;
    int source1 = 0;
    uint64_t source1_tag=0;
    bool source1_renamed=false;
    bool source1_ready=false;
    bool source2_ready=false;
    int source2=0;
    uint64_t source2_tag=0;
    bool source2_renamed=false;

    int age = 0;
};

struct EX{
    bool valid = false;
    int op_type = 0;
    int destination = 0;
    uint64_t destination_tag = 0;
    int source1 = 0;
    uint64_t source1_tag=0;
    bool source1_renamed=false;
    bool source1_ready=false;
    bool source2_ready=false;
    int source2=0;
    uint64_t source2_tag=0;
    bool source2_renamed=false;
    uint64_t timer = 0;
    // bool finished = false;

    int age = 0;
};

struct WB{
    bool valid = false;
    int op_type = 0;
    int destination = 0;
    uint64_t destination_tag = 0;
    int source1 = 0;
    uint64_t source1_tag=0;
    bool source1_renamed=false;
    bool source1_ready=false;
    bool source2_ready=false;
    int source2=0;
    uint64_t source2_tag=0;
    bool source2_renamed=false;

    int age = 0;
};

struct RT{
    bool valid = false;
    int op_type = 0;
    int destination = 0;
    uint64_t destination_tag = 0;
    int source1 = 0;
    uint64_t source1_tag=0;
    bool source1_renamed=false;
    bool source1_ready=false;
    bool source2_ready=false;
    int source2=0;
    uint64_t source2_tag=0;
    bool source2_renamed=false;

    int age = 0;
};

class superscalar{
    public:
    bool end_of_file = false;
    int head=0,tail=0; // increment whenever entry in ROB
    int width,iq_size,rob_size,ex_width, wb_width;
    uint64_t pc = 0;
    int reduce_latency = -1;
    int invalid = -1;
    int cycles = 0; // increases in advance cycle only
    int instructions_count=0;
    // One way to annotate the age of an instruction is to assign an 
    // incrementing sequence number to each instruction as it is fetched from the trace file. instructions_count variable is there to track
    vector <pipeline_entries> entires;
    vector <pipeline_begin_cycles> begin_cycle; // values used for cycle start and duration
    int OP_LATENCY [3] = {1, 2, 5};

    // vector <RT> retire;
    vector <WB> writeback;
    vector <EX> execute_list;
    vector <DI> dispatch; //dount weather to check readiness
    vector <ROB> rob;
    vector <RMT> rmt;
    vector <IQ> issue_q;
    vector <RR> reg_read;
    vector <RN> rename;
    vector <DE> decode;

    superscalar(int rob_size,int iq_size, int width){
        this->width = width;
        this->iq_size = iq_size;
        this->rob_size = rob_size;
        this->ex_width = width * 5;
        this->wb_width = width * 5;
        entires.resize(20000);
        begin_cycle.resize(20000);
        rob.resize(rob_size);
        rmt.resize(67);
        writeback.resize(wb_width);
        execute_list.resize(ex_width);
        dispatch.resize(width);
        issue_q.resize(iq_size);
        reg_read.resize(width);
        rename.resize(width);
        decode.resize(width);

        for(int i = 0;i<rob_size;i++){
            rob[i].rob_index = i; //initializing rob0,rob1, indexes track back purpose
            rob[i].ready = false;
            rob[i].valid = false;
        }
    }

    // Process the writeback bundle in WB: 
    // For each instruction in WB, mark the 
    // instruction as “ready” in its entry |- then need to check destination with rob in 
    // look slide 78->79 lol 
    // the ROB.
    void Retire() {
        int retired = 0;
        // Only process up to WIDTH instructions
        while(retired < width && rob[head].valid && rob[head].ready) { //for loop timing out, while good

            if(rob[head].destination != invalid) {
                rob[head].valid = false;
                if(rmt[rob[head].destination].valid && rmt[rob[head].destination].tag == rob[head].rob_index) { // this was the last fix 
                rmt[rob[head].destination].valid = false;
                }
            }
            else if(rob[head].destination == invalid){
                rob[head].ready = false;
                rob[head].valid = false;

            }
            else{
                return;
            }    
            // Update timing info
        begin_cycle[rob[head].age].end = cycles + 1;
        // Move head pointer
        head = (head + 1) % rob_size;
        retired++;
        }
    }
    
    void Writeback() {
        // printf("WB srt\n");
        int i = 0;
        while(i < wb_width) {
            if(writeback[i].valid) {
                // find matching ROB entry directly using destination_tag
                if(rob[writeback[i].destination_tag].valid) {
                    // mark instruction as ready in ROB
                    rob[writeback[i].destination_tag].ready = true;
                    rob[writeback[i].destination_tag].age = writeback[i].age;
                    begin_cycle[writeback[i].age].retire = cycles + 1;
                    begin_cycle[writeback[i].age].writeback_duration = begin_cycle[writeback[i].age].retire - begin_cycle[writeback[i].age].writeback;
                    // Remove instruction from writeback
                    writeback[i].valid = false;
                }
            }
            i++;
        }
        // printf("WB end\n");
    }

    void Execute(){
        //first find which instruction is finishing executing this cycle
        // checking if latency = 0
        // printf("Execute\n");
        for(int i = 0;i<ex_width;i++){
            if(execute_list[i].valid){
                if(execute_list[i].timer > 0) {
                    execute_list[i].timer += reduce_latency;
                }
                if(execute_list[i].timer == 0){
                    uint64_t completing_tag = execute_list[i].destination_tag;
                    //first for IQ source 1
                    // 1) Remove the instruction from the execute_list -> just invalid it
                    // execute_list[i].valid = false; // removed 
                    // 2) Add the instruction to WB. Find an empty spot
                    for(int j = 0;j<wb_width;j++){
                        if(!writeback[j].valid){
                            if(execute_list[i].destination != invalid){
                                for(int j = 0; j < iq_size; j++) {
                                    if(issue_q[j].valid) {
                                        if(issue_q[j].source1_tag == completing_tag) {
                                            issue_q[j].source1_ready = true;
                                            issue_q[j].source1_renamed = true;
                                        }
                                        if(issue_q[j].source2_tag == completing_tag) {
                                            issue_q[j].source2_ready = true;
                                            issue_q[j].source2_renamed = true;
                                        }
                                    }
                                }//iq waking out done
                                for(int k = 0; k < width; k++) { // avoiding timeout since width same anyway
                                    if(dispatch[k].valid) {
                                        if(dispatch[k].source1_tag == completing_tag) {
                                            dispatch[k].source1_ready = true;
                                        }
                                        if(dispatch[k].source2_tag == completing_tag) {
                                            dispatch[k].source2_ready = true;
                                        }
                                    }
                                    if(reg_read[k].valid) {
                                        if(reg_read[k].source1_tag == completing_tag) {
                                            reg_read[k].source1_ready = true;
                                        }
                                        if(reg_read[k].source2_tag == completing_tag) {
                                            reg_read[k].source2_ready = true;
                                        }
                                    }
                                } // DI, RR waking up done
                            }
                            // while waking up after moving to ex stage, timing out wrong values, so DO IT BEFORE lol (took 4 days)
                            //move to EX
                            execute_list[i].valid = false;
                            writeback[j].valid = true;
                            writeback[j].age = execute_list[i].age; // not sure this time due to timer, maybe execut.age
                            writeback[j].destination = execute_list[i].destination;
                            writeback[j].destination_tag = execute_list[i].destination_tag;
                            writeback[j].op_type = execute_list[i].op_type;
                            writeback[j].source1 = execute_list[i].source1;
                            writeback[j].source1_renamed = execute_list[i].source1_renamed;
                            writeback[j].source1_ready = execute_list[i].source1_ready;
                            writeback[j].source1_tag = execute_list[i].source1_tag;

                            writeback[j].source2 = execute_list[i].source2;
                            writeback[j].source2_renamed = execute_list[i].source2_renamed;
                            writeback[j].source2_ready = execute_list[i].source2_ready;
                            writeback[j].source2_tag = execute_list[i].source2_tag;
                            begin_cycle[writeback[j].age].writeback = cycles + 1;
                            begin_cycle[writeback[j].age].execute_duration = begin_cycle[writeback[j].age].writeback - begin_cycle[execute_list[i].age].execute;
                            break;
                        }
                    }
                    // 3) Wakeup dependent instructions (set their source operand ready flags) in 
                    // the IQ, DI (the dispatch bundle), and RR (the register-read bundle)
                    // compare this->destination with that_stage->source1/2 if not ready make it ready
                }
            }
        }
        // printf("EX END\n");
        return;
    }

    void Issue() {
        // printf("IS start\n");
        vector<pair<int, int>> vec; // pair of <age, index>
        //Same like dic in python. When used just 1D vec didnt work because of same age probably
        //THis safer
        // checking readiness of all instructions in IQ
        // Issue up to WIDTH oldest instructions from the IQ
        // head, keep on incrementing head: oldest - in rob
        //create an array of that size(number of instructions that are valid and are ready for execution in IQ bundle)
        //store all the valid ready elements(age parameter) in the array
        int iq_counts = 0;
        for(int i = 0; i < (static_cast<int>(issue_q.size())); i++) {
            if(issue_q[i].valid && issue_q[i].source1_ready && issue_q[i].source2_ready){
                iq_counts++;
                vec.push_back({issue_q[i].age, i});
                }
            }
        
            // Sort by age to maintain oldest-first policy
            sort(vec.begin(), vec.end());
                    
            // printf("%d\n",iq_counts);
            // Issue up to WIDTH instructions
            int issued = 0;
            // printf("Works\n");
            // printf("%d %d\n",static_cast<int>(vec.size()),width);
            for(auto [age, idx] : vec) {
                if(issued >= min(iq_counts,width))
                    break;
                else{
                // Find empty spot in execute_list
                    for(int k = 0; k < ex_width; k++) {
                        if(!execute_list[k].valid) {
                        // removing the instruction from IQ
                        // see nov 18 class at 39:30 then why valid = 0 lol
                        // after it goes to next stage it wont be in IQ
                        // adding the instruction to ex
                        execute_list[k].valid = true;
                        execute_list[k].age = issue_q[idx].age;
                        execute_list[k].destination = issue_q[idx].destination;
                        execute_list[k].destination_tag = issue_q[idx].destination_tag;
                        execute_list[k].op_type = issue_q[idx].op_type;
                        execute_list[k].source1 = issue_q[idx].source1;
                        execute_list[k].source1_renamed = issue_q[idx].source1_renamed;
                        execute_list[k].source1_ready = issue_q[idx].source1_ready;
                        execute_list[k].source1_tag = issue_q[idx].source1_tag;
                        execute_list[k].source2 = issue_q[idx].source2;
                        execute_list[k].source2_renamed = issue_q[idx].source2_renamed;
                        execute_list[k].source2_ready = issue_q[idx].source2_ready;
                        execute_list[k].source2_tag = issue_q[idx].source2_tag;
                        execute_list[k].timer = OP_LATENCY[execute_list[k].op_type]; //  will allow you to model its execution latency.
                        begin_cycle[execute_list[k].age].execute = cycles + 1;
                        begin_cycle[execute_list[k].age].issue_duration = begin_cycle[execute_list[k].age].execute - begin_cycle[issue_q[idx].age].issue;
                        issue_q[idx].valid = false;
                        issued++;
                        break; // now searching for next free space
                        }
                    }
                }
            }
            // printf("Issue end\n");
                }
            // valid_iq_counter = valid_iq_counter - 1;
        // }
    
    void Dispatch(){
        // int invalid_value = invalid;
        // if(cycles == 528){
        //     printf("Issue queue\n");
        //     for(int i = 0;i<iq_size;i++){
        //         printf("%d %d %d %d\n",issue_q[i].valid,issue_q[i].destination,issue_q[i].source1,issue_q[i].source2);
        //     }
        // }
        bool dispatch_empty = true;
        int dispatch_count = 0;
        for(int i = 0;i<width;i++){
            if(dispatch[i].valid){   
                dispatch_count++;
            }
        }

        // check for readyness?
        int free_entries = 0;
        bool enough_entries = false;
            //  If the number of free IQ entries is less do nothing
        for(int i = 0;i<iq_size;i++){
            if(!issue_q[i].valid){
                free_entries++;
            }
        }
        if(free_entries == 0){
            return;
        }

        dispatch_empty = dispatch_count > 0;
        enough_entries = free_entries >= dispatch_count;
        // printf("Dispatch start\n");
        if(dispatch_empty){
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
                                issue_q[j].source1_renamed = dispatch[i].source1_renamed;
                                issue_q[j].source1_tag = dispatch[i].source1_tag;
                                issue_q[j].source1_ready = dispatch[i].source1_ready;
                                issue_q[j].source2 = dispatch[i].source2;
                                issue_q[j].source2_renamed = dispatch[i].source2_renamed;
                                issue_q[j].source2_tag = dispatch[i].source2_tag;
                                issue_q[j].source2_ready = dispatch[i].source2_ready;
                                dispatch[i].valid = false; // removed instruction
                                begin_cycle[issue_q[j].age].issue = cycles + 1;
                                begin_cycle[issue_q[j].age].dispatch_duration = begin_cycle[issue_q[j].age].issue - begin_cycle[dispatch[i].age].dispatch;
                                break; // breaks out of for j loop since its valid now looking for another spot
                            }
                        }
                    }
                }
            }
        }
        // printf("Dispatch end\n");

    }

    void RegRead(){
        // printf("RedREad start\n");
        // If RR contains a register-read bundle:
        bool bundle_present = false;
        bool dispatch_empty = true;
        int invalid_value = invalid;
        
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
            }
        }

        /*
        
         If DI is empty and bundle present
        // Since values are not explicitly modeled, the sole purpose of the Register Read 
        // stage is to ascertain the readiness of the renamed source operands. Always putting sourcei.ready = true
        */
       /*
        - For any valid source register, readiness as evaluated in RegRead() depends on two factors: 
        (1) which register file partition it was renamed to in Rename() (either the ARF -- whose committed values are always ready -- or the ROB) and
         (2) if it was renamed to the ROB (linked to a producer in the ROB), the producer's ROB entry has a ready bit that is managed by the producer (it may or may not be ready yet, depending on if and when the producer completed).
         This evaluation of readiness based on the above two factors was exhibited in various scenarios in the detailed simulation from class.
         */
        // Readiness of each source register must be individually ascertained.  That should be the context for interpreting my previous reply.
         // An instruction's overall readiness is evaluated by considering readiness of all its source registers.
         //if valid instruction
        // order is rmt s1->rob s1-tag->make it ready bruh
        // check if source valid in rmt
        // if yes then using rob tag check if ready in rob

       if(bundle_present && dispatch_empty){
            for(int i = 0;i<width;i++){
                if(reg_read[i].valid){
                    // check if valid source | if not ready then set it ready                 
                    int temp1 = reg_read[i].source1;
                    if(temp1 != invalid_value){
                        //check rmt for this source if valid then it is not executed yet
                        // then also set ready
                        if(rmt[temp1].valid){
                            // check if destination is the source; and valid in rob and ready
                            for(int j = 0;j<rob_size;j++){
                                if(rob[j].valid && rob[reg_read[i].source1_tag].ready && ( reg_read[i].source1_renamed)){
                                    reg_read[i].source1_ready = true;
                                    break;
                                }
                            }
                        }
                        else{ // not in rmt then in arf so ready
                            reg_read[i].source1_ready = true;
                        }
                        // set ready anyway. should we also set ready if no source? ask Prof: said yes or omit it
                    }
                    else{ // make it truw so that it doesnt cause delay
                        reg_read[i].source1_ready = true;
                    }

                    // same for s2
                    int temp2 = reg_read[i].source2;
                    if(temp2 != invalid_value){
                        //check rmt for this source if valid then it is not executed yet
                        // then also set ready
                        if(rmt[temp2].valid){
                            for(int j = 0;j<rob_size;j++){
                                if(rob[j].valid && rob[reg_read[i].source2_tag].ready && (reg_read[i].source2_renamed)){
                                    reg_read[i].source2_ready = true;
                                    break;
                                }
                            }
                        }
                        else{ //invalid
                            reg_read[i].source2_ready = true;
                        }
                        // set ready anyway. should we also set ready if no source? ask Prof
                    }
                    else if(temp2 == invalid_value){
                        reg_read[i].source2_ready = true; //set anyway to avoid delay
                    }
                    // process (see below) the register-read bundle and advance it from RR to DI. we look for empty place in di
                    bool empty_spot = !dispatch[i].valid;
                    if(empty_spot && reg_read[i].valid){
                        dispatch[i].age = reg_read[i].age;
                        dispatch[i].valid = true; // true
                        dispatch[i].op_type = reg_read[i].op_type;
                        dispatch[i].destination = reg_read[i].destination;
                        dispatch[i].destination_tag = reg_read[i].destination_tag;
                        dispatch[i].source1 = reg_read[i].source1;
                        dispatch[i].source1_ready = reg_read[i].source1_ready;
                        dispatch[i].source1_tag = reg_read[i].source1_tag;
                        dispatch[i].source1_renamed = reg_read[i].source1_renamed;
                        dispatch[i].source2 = reg_read[i].source2;
                        dispatch[i].source2_ready = reg_read[i].source2_ready;
                        dispatch[i].source2_tag = reg_read[i].source2_tag;
                        dispatch[i].source2_renamed = reg_read[i].source2_renamed;
                        reg_read[i].valid = false; // removed instruction
                        begin_cycle[dispatch[i].age].dispatch = cycles + 1;
                        begin_cycle[dispatch[i].age].reg_read_duration = begin_cycle[dispatch[i].age].dispatch - begin_cycle[dispatch[i].age].reg_read;
                    }
                }
            }
       }
    //    printf("RedREad end\n");

    }

    void Rename(uint64_t PC){
        // printf("RN start\n");
        // If RN contains a rename bundle: 
        bool bundle_present = true;
        int rename_counts = 0;
        int invalid_value = invalid; // if equal to this do nothing

        for(int i = 0;i<width;i++){
            if(!rename[i].valid){
                bundle_present = false;
                return;
            }
            if(rename[i].valid){
                rename_counts++;
            }
        }
        // printf("Rename Count %d\n",rename_counts);
        //  RR is empty assume
        bool rr_empty = true;
        for(int i = 0;i<width;i++){
            if(reg_read[i].valid){
                rr_empty = false; //if atleast one not empty do nothing
                return;
            }
        }

        if(bundle_present){ // // If RN contains a rename bundle: 
            //the ROB has enough free entries >= width
            int rob_counter = 0;
            for(int i = 0;i<rob_size;i++){
                if(!rob[i].valid){
                    rob_counter++;
                }
            }
            bool enough_spaces_in_rob = rob_counter >= width;
            // printf("Working till here\n"); // works
            if(enough_spaces_in_rob && rr_empty){
                for(int i = 0;i<width;i++){
                    if(rename[i].valid){ // only change for valid instr
                        // printf("rename : %d %d %d %d\n",rename[i].op_type,rename[i].destination,rename[i].source1,rename[i].source2);
                        // now ahave to assign dest in rob at tail
                        int temp_destination = rename[i].destination;
                        //1) allocating entry in rob in index tail
                        rob[tail].valid = true;
                        rob[tail].ready = false;
                        rob[tail].destination = rename[i].destination; // allocate it even if no destination but not in rmt
                        rob[tail].pc = PC;
                        rename[i].destination_tag = rob[tail].rob_index;
                        // printf("%d \n",rob[tail].rob_index);
                        // now check source there or not
                        // printf("Working till here\n"); - working
                        // 2) renaming source registers if there; if in rmt then only can rename; save that tag in rob
                        if(rename[i].source1 != invalid_value){
                            // there
                            int temp = rename[i].source1;
                            // printf("Working till here 1 \n"); // yes
                            // printf("Precheck 1\n");
                            if(rmt[rename[i].source1].valid){
                                // printf("after if check 1\n");
                                // printf("Working till here\n"); - nope
                                // present in rob already, waiting for execution
                                rename[i].source1_renamed = true;
                                rename[i].source1_ready = false;
                                rename[i].source1_tag = rmt[temp].tag; //tail
                            }
                            else{
                                // printf("after else check 1\n");
                                // if not waiting then in ARF, set to 0
                                // printf("Working till here\n");
                                rename[i].source1_renamed = false;
                                rename[i].source1_ready = true;
                                rename[i].source1_tag = invalid;
                                // not there in rob
                            }
                            
                            // printf("Working till here 2 \n");
                        }
                        // else{
                        //     rename[i].source1_ready = true;
                        // }
                        // printf("Working till here\n");
                        // printf("Working till here\n"); -not working
                        if(rename[i].source2 != invalid_value){
                            // there
                            int temp = rename[i].source2;
                            if(rmt[temp].valid){
                                // printf("ho\n");
                                // present in rob already, waiting for execution, it is waiting for value in pipeline, not commited version
                                rename[i].source2_renamed = true;
                                rename[i].source2_tag = rmt[temp].tag;
                                rename[i].source2_ready = false;
                            }
                            else{
                                // if not waiting then in ARF, set to 0
                                rename[i].source2_renamed = false;
                                rename[i].source2_ready = true;
                                rename[i].source2_tag = invalid;
                                // not there in rob
                            }
                        }
                        else{
                             rename[i].source2_ready = true;
                        }

                        //  (3) rename its destination register (if it has one)
                        if(rename[i].destination != invalid_value){ //we put branch in rob but not in rmt
                            rmt[rob[tail].destination].valid = true;
                            rmt[rob[tail].destination].tag = rob[tail].rob_index;//tail may change so using index not tail;//
                            rmt[temp_destination].tag = rob[tail].rob_index;
                        } // if branch then do nothing

                        tail = (tail + 1)%rob_size; // since circular if end we start from 0

                        if(!(reg_read[i].valid) && rename[i].valid){
                            // and advance it from RN TO RR
                            reg_read[i].valid = true;
                            reg_read[i].age = rename[i].age;
                            reg_read[i].destination = rename[i].destination;
                            reg_read[i].destination_tag = rename[i].destination_tag;
                            reg_read[i].op_type = rename[i].op_type;
                            reg_read[i].source1 = rename[i].source1;
                            reg_read[i].source1_renamed = rename[i].source1_renamed;
                            reg_read[i].source1_tag = rename[i].source1_tag;
                            reg_read[i].source1_ready = rename[i].source1_ready;
                            reg_read[i].source2 = rename[i].source2;
                            reg_read[i].source2_renamed = rename[i].source2_renamed;
                            reg_read[i].source2_ready = rename[i].source2_ready;
                            reg_read[i].source2_tag = rename[i].source2_tag;
                            rename[i].valid = false; // removed instruction
                            begin_cycle[reg_read[i].age].reg_read = cycles + 1;
                            begin_cycle[reg_read[i].age].rename_duration = begin_cycle[reg_read[i].age].reg_read - begin_cycle[reg_read[i].age].rename;
                        }
                    }
                }
            }
            else{
                return;
            }
        }
        // printf("RN end\n");
    }

    void Decode(){
        // printf("DE start\n");
        bool bundle_present = false;
        bool rename_vacant = true;
        int rename_count = 0;

        for(int i = 0;i<width;i++){
            if(decode[i].valid){
                bundle_present = true;
                break;
            }
        } 

        if(bundle_present){
            for(int i = 0;i < width;i++){
                if(!rename[i].valid){
                    rename_count++; // not empty, do nothing
                    // break;
                }
            }
            // printf("rename_count %d\n",rename_count);
            rename_vacant = rename_count == width;
            // if total empty should we check if decode i valid also
            if (rename_vacant){
                for(int i=0;i < width;i++){
                    if(decode[i].valid && (!rename[i].valid)){
                        // printf("Renaming instruction %d: op_type = %d, destination = %d, source1 = %d, source2 = %d, age = %d\n",
                        // instructions_count, decode[i].op_type, decode[i].destination, decode[i].source1, decode[i].source2, decode[i].age);
                        rename[i].op_type = decode[i].op_type;
                        rename[i].destination = decode[i].destination;
                        rename[i].source1 = decode[i].source1;
                        rename[i].source2 = decode[i].source2;
                        rename[i].age = decode[i].age;
                        rename[i].valid = true;
                        decode[i].valid = false;
                        // rename age leda inst cont
                        begin_cycle[decode[i].age].rename = cycles + 1;
                        begin_cycle[decode[i].age].decode_duration = begin_cycle[decode[i].age].rename - begin_cycle[decode[i].age].decode;
                        // printf("rename  from decode: %d %d %d %d\n",rename[i].op_type,rename[i].destination,rename[i].source1,rename[i].source2);
                    }                    
                }
            }
            
        }
        // printf("DE end\n");
    }

    void Fetch(FILE *FP = NULL){
        // printf("FE start\n");
        // printf("Cycles %d\n",this->cycles);
        bool decode_empty = true;
        int op_type;
        int destination;
        int source1;
        int source2;
        uint64_t pc;
        int decode_count=0;
        for(int i=0;i<width;i++){
            if(!decode[i].valid){
                decode_count++; //not empty
            }
        }
        // printf("Decode count %d\n", decode_count);
        
        if(!(feof(FP)) && decode_count == width){  //instructions in trace file 
            //if DE is empty
            // printf("hi\n");
                for(int i=0;i<width && !end_of_file;i++){    //upto width only
                    fscanf(FP,"%llx %d %d %d %d\n",&pc,&op_type,&destination,&source1,&source2);

                    instructions_count++; // increase after each read from file
                    decode[i].valid = true;
                    decode[i].op_type = op_type;
                    decode[i].destination = destination;
                    decode[i].source1 = source1;
                    decode[i].source2 = source2;
                    decode[i].age=instructions_count;

                    //now putting into serial pipeline
                    entires[instructions_count].instruction_number = decode[i].age;
                    entires[instructions_count].cycles_count = cycles;
                    entires[instructions_count].op_type = op_type;
                    entires[instructions_count].source1 = source1;
                    entires[instructions_count].source2 = source2;
                    entires[instructions_count].destination = destination;
                    entires[instructions_count].valid = true;
                    
                    begin_cycle[instructions_count].fetch = cycles;
                    begin_cycle[instructions_count].decode = cycles + 1;
                    begin_cycle[instructions_count].fetch_duration = 1;

                    if(feof(FP)){
                        end_of_file = true;
                        // printf("End of file reached.\n");
                        break; // no more instructions
                        // return;
                    }
                }
        }
        // printf("FE end\n");
    
    }

    // Advance_Cycle performs several functions.  
    // First, it advances the simulator cycle. 
    // Second, when it becomes known that the  
    // pipeline is empty AND the trace is depleted
    // when is pipeline empty?
    // when it is not empty? -> when any of pipeline registers is busy/valid?
    // the function returns “false” to terminate 
    // the loop.
    bool Advance_Cycle() {
        // printf("%d \n",cycles);
        cycles++; // Advance simulator cycle
        // printf("cycles : %d\n",cycles);
        bool pipeline_empty = true;

        // Checking if pipeline stages have valid instructions
        for(int i = 0; i < width; i++) {
            if(decode[i].valid || rename[i].valid || reg_read[i].valid || dispatch[i].valid){
                pipeline_empty = false;
                break;
            }
        }
        
        // Checking IQ
        for(int i = 0; i < iq_size; i++) {
            if(issue_q[i].valid){
                pipeline_empty = false;
                break;
            }
        }
        
        // Checking EX
        for(int i = 0; i < ex_width; i++) {
            if(execute_list[i].valid){
                pipeline_empty = false;
                break;
            }
        }
        
        // Checking WB
        for(int i = 0; i < wb_width; i++) {
            if(writeback[i].valid){
                pipeline_empty = false;
                break;
            }
        }
        
        // Checking ROB
        for(int i = 0; i < rob_size; i++) {
            if(rob[i].valid){
                pipeline_empty = false;
                break;
            }
        }

        if(!(head == tail && cycles > 0))
            pipeline_empty = false;
            
        // Return false to terminate if pipeline is empty and trace is depleted
        if(pipeline_empty && end_of_file)
            return false;

        return true;
    }
    
    void print_values(){
        for(int i = 1;i<instructions_count+1;i++){
            printf("%d fu{%d} src{%d,%d} dst{%d} FE{%d,%d} DE{%d,%d} RN{%d,%d} RR{%d,%d} DI{%d,%d} IS{%d,%d} EX{%d,%d} WB{%d,%d} RT{%d,%d}\n",
            entires[i].instruction_number-1,
            entires[i].op_type,
            entires[i].source1,
            entires[i].source2,
            entires[i].destination,
            begin_cycle[i].fetch,
            begin_cycle[i].decode - begin_cycle[i].fetch,

            begin_cycle[i].decode,
            begin_cycle[i].rename - begin_cycle[i].decode,

            begin_cycle[i].rename,
            begin_cycle[i].reg_read - begin_cycle[i].rename,

            begin_cycle[i].reg_read,
            begin_cycle[i].dispatch - begin_cycle[i].reg_read,

            begin_cycle[i].dispatch,
            begin_cycle[i].issue - begin_cycle[i].dispatch,

            begin_cycle[i].issue,
            begin_cycle[i].execute - begin_cycle[i].issue,
            begin_cycle[i].execute,
            begin_cycle[i].execute_duration,
            begin_cycle[i].writeback,
            begin_cycle[i].writeback_duration,
            begin_cycle[i].retire,
            begin_cycle[i].end - begin_cycle[i].retire 
            );
        }
    }

};

// One way to annotate the age of an instruction is to assign an 
// incrementing sequence number to each instruction as it is fetched from the trace file. Instruction_count variable is there to track
// The variable age tracks this

int main (int argc, char* argv[])
{
    FILE *FP;               // File handler
    char *trace_file;       // Variable that holds trace file name;
    proc_params params;       // look at sim_bp.h header file for the the definition of struct proc_params
    uint64_t pc; // Variable holds the pc read from input file
    
    if (argc != 5)
    {
        printf("Error: Wrong number of inputs:%d\n", argc-1);
        exit(EXIT_FAILURE);
    }
    pc                  = strtoul(argv[0], NULL, 10);
    params.rob_size     = strtoul(argv[1], NULL, 10);
    params.iq_size      = strtoul(argv[2], NULL, 10);
    params.width        = strtoul(argv[3], NULL, 10);
    trace_file          = argv[4];
    FP = fopen(trace_file, "r");
    if(FP == NULL)
    {
        // Throw error and exit if fopen() failed
        printf("Error: Unable to open file %s\n", trace_file);
        exit(EXIT_FAILURE);
    }
    // from the fetch function we read the tracefile
    superscalar superscalar_pipeline_simulator(params.rob_size,params.iq_size,params.width);
    do{
        superscalar_pipeline_simulator.Retire();
        // printf("RT done \n");
        superscalar_pipeline_simulator.Writeback();
        // printf("WB done \n");
        superscalar_pipeline_simulator.Execute();
        // printf("Execute done\n");
        superscalar_pipeline_simulator.Issue();
        // printf("Issue\n");
        superscalar_pipeline_simulator.Dispatch();
        // printf("Dispatch\n");
        superscalar_pipeline_simulator.RegRead();
        // printf("Regread done\n");
        superscalar_pipeline_simulator.Rename(pc);
        // printf("Rn done\n");
        superscalar_pipeline_simulator.Decode();
        // printf("Decode done\n");
        superscalar_pipeline_simulator.Fetch(FP);
        // printf("Fetch done\n\n");
    }while(superscalar_pipeline_simulator.Advance_Cycle());

    superscalar_pipeline_simulator.print_values();

    float IPC = ((float)superscalar_pipeline_simulator.instructions_count/((float)superscalar_pipeline_simulator.cycles));
    printf("# === Simulator Command =========\n");
    printf("# ./sim %d %d %d %s\n",params.rob_size,params.iq_size,params.width,trace_file);
    printf("# === Processor Configuration ===\n");
    printf("# ROB_SIZE  = %d\n",superscalar_pipeline_simulator.rob_size);
    printf("# IQ_SIZE  = %d\n",superscalar_pipeline_simulator.iq_size);
    printf("# WIDTH    = %d\n",superscalar_pipeline_simulator.width);
    printf("# === Simulation Results ========\n");
    printf("# Dynamic Instruction Count    = %d\n",superscalar_pipeline_simulator.instructions_count);
    printf("# Cycles                       = %d\n",superscalar_pipeline_simulator.cycles);
    printf("# Instructions Per Cycle (IPC) = %.2f\n",IPC);

    return 0;
}
