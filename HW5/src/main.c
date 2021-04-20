#include "main.h"

#define BOUNDARY_CAP -100000
#define BOUNDARY_PENALTY 100000
#define ROUTED_PENALTY 100000
#define MAZE_INF 100000

int main(int argc, char *argv[]){
    FILE *nets_file, *out_file;

    int GRID_x, GRID_y, V_CAP, H_CAP, NUM_NETs;
    int net_name, net_id, num_pin, pin_x, pin_y;

    NET **NETs_arr;
    NET *new_net;

    BIN **BINs_arr;
    BIN *new_bin;

    CPOINT *curr_point;
    CPOINT *new_point;
    CPOINT *rip_point;
    CPOINT *maze_list;
    CPOINT *maze_curr;
    CPOINT *maze_tile;
    CPOINT *maze_new;
    CPOINT *maze_search;

    int i, j, k, cnt = 1, sign;
    int curr_x, curr_y;
    float cost[4], space[4], cong[4], manh[4]; /* T==0, B==1, L==2, R==3 */
    int source = 0, sink = 1;
    int total_wirelength = 0, total_overflow = 0, overflow = 0;
    int total_manh_x = 0, total_manh_y = 0;
    int route_dir;
    float alpha, beta, step, alpha_cap, step_ratio;
    float terminate_ratio = 0.1;
    int iteration = 4;

    int *Sorted_Nets;
    int sort_tmp;
    float gamma;
    float cmp_1, cmp_2;

    int maze_x, maze_y, maze_ov, maze_cnt = 0;

    clock_t begin, end, route_begin, route_end, total_begin, total_end;

    total_begin = clock();

    if(argc != 3){
		printf("\n Error. Takes 2 arguments\n");
		return -1;
	}

    nets_file = R_file(argv[1], "r");
    if(nets_file == NULL) return -1;

    out_file = R_file(argv[2], "w");
    if(out_file == NULL) return -1;

	printf("Open files\n");
    
    begin = clock();

    fscanf(nets_file, "grid %d %d\n", &GRID_x, &GRID_y);  
    fscanf(nets_file, "vertical capacity %d\n", &V_CAP); 
    fscanf(nets_file, "horizontal capacity %d\n", &H_CAP);
    fscanf(nets_file, "num net %d\n", &NUM_NETs);

    printf("Build Bins Array\n");
    BINs_arr = malloc(GRID_x*GRID_y*sizeof(BIN *));
    for(i=0; i<GRID_x; ++i){
        for(j=0; j<GRID_y; ++j){
            new_bin = malloc(sizeof(BIN));
            new_bin->cor_x = i;
            new_bin->cor_y = j;

            if(j==(GRID_y-1)) new_bin->T_cap = BOUNDARY_CAP;
            else new_bin->T_cap = H_CAP;

            if(j==0) new_bin->B_cap = BOUNDARY_CAP;
            else new_bin->B_cap = H_CAP;

            if(i==0) new_bin->L_cap = BOUNDARY_CAP; 
            else new_bin->L_cap = V_CAP;

            if(i==(GRID_x-1)) new_bin->R_cap = BOUNDARY_CAP; 
            else new_bin->R_cap = V_CAP;

            new_bin->T_occ = 0;
            new_bin->B_occ = 0;
            new_bin->L_occ = 0;
            new_bin->R_occ = 0;

            new_bin->T_hp = 1;
            new_bin->B_hp = 1;
            new_bin->L_hp = 1;
            new_bin->R_hp = 1;

            new_bin->maze_cost = MAZE_INF;
            new_bin->maze_flag = false;

            new_bin->routed = false;            

            BINs_arr[i*GRID_y+j] = new_bin;
        }
    }

    for(i=0; i<GRID_x; ++i){
        for(j=0; j<GRID_y; ++j){
            if(i==0) BINs_arr[i*GRID_y+j]->L_next = NULL;
            else BINs_arr[i*GRID_y+j]->L_next = BINs_arr[(i-1)*GRID_y+j];

            if(i==(GRID_x-1)) BINs_arr[i*GRID_y+j]->R_next = NULL;
            else BINs_arr[i*GRID_y+j]->R_next = BINs_arr[(i+1)*GRID_y+j];

            if(j==0) BINs_arr[i*GRID_y+j]->T_next = NULL;
            else BINs_arr[i*GRID_y+j]->T_next = BINs_arr[i*GRID_y+(j-1)];

            if(j==(GRID_y-1)) BINs_arr[i*GRID_y+j]->B_next = NULL;
            else BINs_arr[i*GRID_y+j]->B_next = BINs_arr[i*GRID_y+(j+1)];
        }
    }

    printf("Read Nets\n");
    NETs_arr = malloc(NUM_NETs*sizeof(NET *));
    while(!feof(nets_file)){
        fscanf(nets_file, "net%d %d %d\n", &net_name, &net_id, &num_pin);
        /* printf("net%d %d %d\n", net_name, net_id, num_pin); */
        new_net = malloc(sizeof(NET));        
        new_net->net_id = net_id;
        
        fscanf(nets_file, " %d %d\n", &pin_x, &pin_y);
        new_net->pin_x[0] = pin_x;
        new_net->pin_y[0] = pin_y;

        fscanf(nets_file, " %d %d\n", &pin_x, &pin_y);
        new_net->pin_x[1] = pin_x;
        new_net->pin_y[1] = pin_y;

        new_net->manh_dis = abs(new_net->pin_x[0] - new_net->pin_x[1]) + abs(new_net->pin_y[0] - new_net->pin_y[1]);
        total_manh_x += abs(new_net->pin_x[0] - new_net->pin_x[1]);
        total_manh_y += abs(new_net->pin_y[0] - new_net->pin_y[1]);

        new_net->congested = false;
        new_net->congest_cnt = 0;
        
        new_point = malloc(sizeof(CPOINT));
        new_point->cor_x = new_net->pin_x[0];
        new_point->cor_y = new_net->pin_y[0];
        new_point->next = NULL;
        new_point->prev = NULL;
        new_net->route = new_point;

        NETs_arr[net_id] = new_net;
    }

    end = clock();
    printf("Time(Read file) : %0.1f\n", (double)(end - begin) / CLOCKS_PER_SEC);

    Sorted_Nets = malloc(NUM_NETs*sizeof(int));

    route_begin = clock();

    alpha_cap = 0.98;
    step = 0.01;
    step_ratio = 2;
    beta = 0.01;
    gamma = 0.1;
    printf("Route\n");    
    /*  routing part    */
    /*  cost = congestion cost + distance cost  */
    /*  congection cost = current penalty(occ>cap then occ-cap else 0)*historical penalty(occ>cap then +1 else keep)  */
    for(i=0; i<NUM_NETs; ++i){
        Sorted_Nets[i] = i;
    }
    while(1){
        /*  sort Nets_arr according to distance and congestion  */
        begin = clock();        

        /*  sort nets with cost */
        for(i=NUM_NETs-1; i>=0; --i){
            for(j=0; j<i; ++j){
                cmp_1 = (1-gamma)*NETs_arr[Sorted_Nets[j]]->congest_cnt + gamma*NETs_arr[Sorted_Nets[j]]->manh_dis;
                cmp_2 = (1-gamma)*NETs_arr[Sorted_Nets[j+1]]->congest_cnt + gamma*NETs_arr[Sorted_Nets[j+1]]->manh_dis;

                if(cmp_1 > cmp_2){
                    sort_tmp = Sorted_Nets[j];
                    Sorted_Nets[j] = Sorted_Nets[j+1];
                    Sorted_Nets[j+1] = sort_tmp;
                }
                else if(cmp_1==cmp_2 && NETs_arr[Sorted_Nets[j]]->congest_cnt > NETs_arr[Sorted_Nets[j+1]]->congest_cnt){
                    sort_tmp = Sorted_Nets[j];
                    Sorted_Nets[j] = Sorted_Nets[j+1];
                    Sorted_Nets[j+1] = sort_tmp;
                }
            }
        }

        end = clock();
        printf("Time(Sort Nets) : %0.1f\n", (double)(end - begin) / CLOCKS_PER_SEC);

        begin = clock();

        /*  route for each net  */
        printf("Iteration : %d\n", cnt);
        for(i=0; i<NUM_NETs; ++i){
            /* printf("Net : %d, Manhattan Distance : %d\n", NETs_arr[i]->net_id, NETs_arr[i]->manh_dis); */
            /* alpha = 1 - ((int) NETs_arr[Sorted_Nets[i]]->manh_dis/step_ratio)*step; */
            alpha = 0.8;
            
            curr_point = NETs_arr[Sorted_Nets[i]]->route;
            /* printf("Iteration : %d, Net %d\n", cnt, i); */
            route_dir = 0;

            NETs_arr[Sorted_Nets[i]]->congested = false;
            NETs_arr[Sorted_Nets[i]]->congest_cnt = 0;
            /*  check congest   */
            rip_point = NETs_arr[Sorted_Nets[i]]->route->next;
            while(rip_point!=NULL){
                if(rip_point->cor_x == rip_point->prev->cor_x){
                    sign = (rip_point->prev->cor_y-rip_point->cor_y)/abs(rip_point->prev->cor_y-rip_point->cor_y);
                    for(k=0; k < abs(rip_point->prev->cor_y-rip_point->cor_y); ++k){
                        if(sign>0){
                            if(BINs_arr[rip_point->prev->cor_x*GRID_y+(rip_point->prev->cor_y-k)]->B_occ > BINs_arr[rip_point->prev->cor_x*GRID_y+(rip_point->prev->cor_y-k)]->B_cap){
                                NETs_arr[Sorted_Nets[i]]->congested = true;
                                NETs_arr[Sorted_Nets[i]]->congest_cnt += BINs_arr[rip_point->prev->cor_x*GRID_y+(rip_point->prev->cor_y-k)]->B_occ - BINs_arr[rip_point->prev->cor_x*GRID_y+(rip_point->prev->cor_y-k)]->B_cap;
                            }
                            if(BINs_arr[rip_point->prev->cor_x*GRID_y+(rip_point->prev->cor_y-(k+1))]->T_occ > BINs_arr[rip_point->prev->cor_x*GRID_y+(rip_point->prev->cor_y-(k+1))]->T_cap){
                                NETs_arr[Sorted_Nets[i]]->congested = true;
                                NETs_arr[Sorted_Nets[i]]->congest_cnt += BINs_arr[rip_point->prev->cor_x*GRID_y+(rip_point->prev->cor_y-(k+1))]->T_occ - BINs_arr[rip_point->prev->cor_x*GRID_y+(rip_point->prev->cor_y-(k+1))]->T_cap;
                            }
                        }
                        else if(sign<0){
                            if(BINs_arr[rip_point->prev->cor_x*GRID_y+(rip_point->prev->cor_y+k)]->T_occ > BINs_arr[rip_point->prev->cor_x*GRID_y+(rip_point->prev->cor_y+k)]->T_cap){
                                NETs_arr[Sorted_Nets[i]]->congested = true;
                                NETs_arr[Sorted_Nets[i]]->congest_cnt += BINs_arr[rip_point->prev->cor_x*GRID_y+(rip_point->prev->cor_y+k)]->T_occ - BINs_arr[rip_point->prev->cor_x*GRID_y+(rip_point->prev->cor_y+k)]->T_cap;
                            }
                            if(BINs_arr[rip_point->prev->cor_x*GRID_y+(rip_point->prev->cor_y+(k+1))]->B_occ > BINs_arr[rip_point->prev->cor_x*GRID_y+(rip_point->prev->cor_y+(k+1))]->B_occ){
                                NETs_arr[Sorted_Nets[i]]->congested = true;
                                NETs_arr[Sorted_Nets[i]]->congest_cnt += BINs_arr[rip_point->prev->cor_x*GRID_y+(rip_point->prev->cor_y+(k+1))]->B_occ - BINs_arr[rip_point->prev->cor_x*GRID_y+(rip_point->prev->cor_y+(k+1))]->B_occ;
                            }
                        }                        
                    }
                }
                else if(rip_point->cor_y == rip_point->prev->cor_y){
                    sign = (rip_point->prev->cor_x-rip_point->cor_x)/abs(rip_point->prev->cor_x-rip_point->cor_x);
                    for(k=0; k < abs(rip_point->prev->cor_x-rip_point->cor_x); ++k){
                        if(sign>0){
                            if(BINs_arr[(rip_point->prev->cor_x-k)*GRID_y+rip_point->prev->cor_y]->L_occ > BINs_arr[(rip_point->prev->cor_x-k)*GRID_y+rip_point->prev->cor_y]->L_cap){
                                NETs_arr[Sorted_Nets[i]]->congested = true;
                                NETs_arr[Sorted_Nets[i]]->congest_cnt += BINs_arr[(rip_point->prev->cor_x-k)*GRID_y+rip_point->prev->cor_y]->L_occ - BINs_arr[(rip_point->prev->cor_x-k)*GRID_y+rip_point->prev->cor_y]->L_cap;
                            }
                            if(BINs_arr[(rip_point->prev->cor_x-(k+1))*GRID_y+rip_point->prev->cor_y]->R_occ > BINs_arr[(rip_point->prev->cor_x-(k+1))*GRID_y+rip_point->prev->cor_y]->R_cap){
                                NETs_arr[Sorted_Nets[i]]->congested = true;
                                NETs_arr[Sorted_Nets[i]]->congest_cnt += BINs_arr[(rip_point->prev->cor_x-(k+1))*GRID_y+rip_point->prev->cor_y]->R_occ - BINs_arr[(rip_point->prev->cor_x-(k+1))*GRID_y+rip_point->prev->cor_y]->R_cap;
                            }
                        }
                        else if(sign<0){
                            if(BINs_arr[(rip_point->prev->cor_x+k)*GRID_y+rip_point->prev->cor_y]->R_occ > BINs_arr[(rip_point->prev->cor_x+k)*GRID_y+rip_point->prev->cor_y]->R_cap){
                                NETs_arr[Sorted_Nets[i]]->congested = true;
                                NETs_arr[Sorted_Nets[i]]->congest_cnt += BINs_arr[(rip_point->prev->cor_x+k)*GRID_y+rip_point->prev->cor_y]->R_occ - BINs_arr[(rip_point->prev->cor_x+k)*GRID_y+rip_point->prev->cor_y]->R_cap;
                            }
                            if(BINs_arr[(rip_point->prev->cor_x+(k+1))*GRID_y+rip_point->prev->cor_y]->L_occ > BINs_arr[(rip_point->prev->cor_x+(k+1))*GRID_y+rip_point->prev->cor_y]->L_cap){
                                NETs_arr[Sorted_Nets[i]]->congested = true;
                                NETs_arr[Sorted_Nets[i]]->congest_cnt += BINs_arr[(rip_point->prev->cor_x+(k+1))*GRID_y+rip_point->prev->cor_y]->L_occ - BINs_arr[(rip_point->prev->cor_x+(k+1))*GRID_y+rip_point->prev->cor_y]->L_cap;
                            }
                        }                        
                    }
                }   

                rip_point = rip_point->next;             
            }
            if(cnt!=1 && !NETs_arr[Sorted_Nets[i]]->congested) continue;

            /*  occupy fix  */
            /* printf("Occupy Adopt\n"); */
            rip_point = NETs_arr[Sorted_Nets[i]]->route->next;
            while(rip_point!=NULL){
                if(rip_point->cor_x == rip_point->prev->cor_x){
                    sign = (rip_point->prev->cor_y-rip_point->cor_y)/abs(rip_point->prev->cor_y-rip_point->cor_y);
                    for(k=0; k < abs(rip_point->prev->cor_y-rip_point->cor_y); ++k){
                        if(sign>0){
                            BINs_arr[rip_point->prev->cor_x*GRID_y+(rip_point->prev->cor_y-k)]->B_occ--;
                            BINs_arr[rip_point->prev->cor_x*GRID_y+(rip_point->prev->cor_y-(k+1))]->T_occ--;
                        }
                        else if(sign<0){
                            BINs_arr[rip_point->prev->cor_x*GRID_y+(rip_point->prev->cor_y+k)]->T_occ--;
                            BINs_arr[rip_point->prev->cor_x*GRID_y+(rip_point->prev->cor_y+(k+1))]->B_occ--;
                        }                        
                    }
                }
                else if(rip_point->cor_y == rip_point->prev->cor_y){
                    sign = (rip_point->prev->cor_x-rip_point->cor_x)/abs(rip_point->prev->cor_x-rip_point->cor_x);
                    for(k=0; k < abs(rip_point->prev->cor_x-rip_point->cor_x); ++k){
                        if(sign>0){
                            BINs_arr[(rip_point->prev->cor_x-k)*GRID_y+rip_point->prev->cor_y]->L_occ--;
                            BINs_arr[(rip_point->prev->cor_x-(k+1))*GRID_y+rip_point->prev->cor_y]->R_occ--;
                        }
                        else if(sign<0){
                            BINs_arr[(rip_point->prev->cor_x+k)*GRID_y+rip_point->prev->cor_y]->R_occ--;
                            BINs_arr[(rip_point->prev->cor_x+(k+1))*GRID_y+rip_point->prev->cor_y]->L_occ--;
                        }                        
                    }
                }   

                rip_point = rip_point->next;             
            }

            /*  rip up  */            
            while(NETs_arr[Sorted_Nets[i]]->route->next!=NULL){       
                /* printf("Rip Up\n");  */
                rip_point = NETs_arr[Sorted_Nets[i]]->route->next;
                NETs_arr[Sorted_Nets[i]]->route->next = rip_point->next;
                if(rip_point->next!=NULL) rip_point->next->prev = rip_point->prev;

                free(rip_point);
            }

            /*  Maze Route  */
            maze_list = malloc(sizeof(CPOINT));
            maze_list->cor_x = NETs_arr[Sorted_Nets[i]]->pin_x[source];
            maze_list->cor_y = NETs_arr[Sorted_Nets[i]]->pin_y[source];
            maze_list->next = NULL;
            maze_list->prev = NULL;

            maze_curr = maze_list;
            maze_tile = maze_list;
            BINs_arr[maze_curr->cor_x*GRID_y+maze_curr->cor_y]->maze_cost = 0;

            int var_h = 8, var_k = 10;

            while(maze_curr!=NULL){                
                if(maze_curr->cor_y+1<GRID_y){
                    /*  In  */
                    /*  update maze cost    */                    
                    /* if(BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y+1)]->B_occ > BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y+1)]->B_cap){
                        maze_ov = BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y+1)]->B_occ-BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y+1)]->B_cap;}
                    else maze_ov = 0;
                    maze_ov = 1 + var_h/(1+pow(2.71, -1*var_k*(BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y+1)]->B_occ-BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y+1)]->B_cap))); */

                    /* printf("Next : %d, Curr : %d\n", BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y+1)]->maze_cost+maze_ov, BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y)]->maze_cost); */
                    /* if(BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y+1)]->maze_cost+maze_ov < BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y)]->maze_cost){
                        BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y)]->maze_cost = BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y+1)]->maze_cost + maze_ov;
                    } */

                    /*  Out */
                    /*  update maze cost    */
                    maze_ov = 1 + var_h/(1+pow(2.71, -1*var_k*(BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y)]->T_occ-BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y)]->T_cap)));

                    /* printf("Next : %d, Curr : %d\n", BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y+1)]->maze_cost+maze_ov, BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y)]->maze_cost); */
                    if(BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y)]->maze_cost+maze_ov < BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y+1)]->maze_cost){
                        BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y+1)]->maze_cost = BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y)]->maze_cost+maze_ov;
                    }

                    /*  add bin */
                    if(!BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y+1)]->maze_flag){
                        BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y+1)]->maze_flag = true;

                        maze_new = malloc(sizeof(CPOINT));
                        maze_new->cor_x = maze_curr->cor_x;
                        maze_new->cor_y = maze_curr->cor_y+1;

                        /*  search from curr    */
                        /* maze_search = maze_curr;
                        while(maze_search!=NULL){
                            if(BINs_arr[maze_new->cor_x*GRID_y+(maze_new->cor_y)]->maze_cost < BINs_arr[maze_search->cor_x*GRID_y+(maze_search->cor_y)]->maze_cost){
                                maze_new->next = maze_search;
                                maze_new->prev = maze_search->prev;
                                maze_search->prev->next = maze_new;
                                maze_search->prev = maze_new;
                                break;
                            }
                            maze_search = maze_search->next;
                        }
                        if(maze_search==NULL){
                            maze_new->next = NULL;
                            maze_new->prev = maze_tile;

                            maze_tile->next = maze_new;
                            maze_tile = maze_tile->next;
                        } */

                        maze_new->next = NULL;
                        maze_new->prev = maze_tile;

                        maze_tile->next = maze_new;
                        maze_tile = maze_tile->next;                  
                    }
                }

                if(maze_curr->cor_y-1>=0){
                    /*  In  */
                    /*  update maze cost    */
                    /* if(BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y-1)]->T_occ > BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y-1)]->T_cap){
                        maze_ov = BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y-1)]->T_occ-BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y-1)]->T_cap;}
                    else maze_ov = 0;
                    maze_ov = 1 + var_h/(1+pow(2.71, -1*var_k*(BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y-1)]->T_occ-BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y-1)]->T_cap))); */

                    /* printf("Next : %d, Curr : %d\n", BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y-1)]->maze_cost+maze_ov, BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y)]->maze_cost); */
                    /* if(BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y-1)]->maze_cost+maze_ov < BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y)]->maze_cost){
                        BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y)]->maze_cost = BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y-1)]->maze_cost + maze_ov;
                    } */

                    /*  Out */
                    /*  update maze cost    */
                    maze_ov = 1 + var_h/(1+pow(2.71, -1*var_k*(BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y)]->B_occ-BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y)]->B_cap)));

                    /* printf("Next : %d, Curr : %d\n", BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y+1)]->maze_cost+maze_ov, BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y)]->maze_cost); */
                    if(BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y)]->maze_cost+maze_ov < BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y-1)]->maze_cost){
                        BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y-1)]->maze_cost = BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y)]->maze_cost+maze_ov;
                    }

                    /*  add bin */
                    if(!BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y-1)]->maze_flag){
                        BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y-1)]->maze_flag = true;

                        maze_new = malloc(sizeof(CPOINT));
                        maze_new->cor_x = maze_curr->cor_x;
                        maze_new->cor_y = maze_curr->cor_y-1;

                        /*  search from curr    */
                        /* maze_search = maze_curr;
                        while(maze_search!=NULL){
                            if(BINs_arr[maze_new->cor_x*GRID_y+(maze_new->cor_y)]->maze_cost < BINs_arr[maze_search->cor_x*GRID_y+(maze_search->cor_y)]->maze_cost){
                                maze_new->next = maze_search;
                                maze_new->prev = maze_search->prev;
                                maze_search->prev->next = maze_new;
                                maze_search->prev = maze_new;
                                break;
                            }
                            maze_search = maze_search->next;
                        }
                        if(maze_search==NULL){
                            maze_new->next = NULL;
                            maze_new->prev = maze_tile;

                            maze_tile->next = maze_new;
                            maze_tile = maze_tile->next;
                        } */                        

                        maze_new->next = NULL;
                        maze_new->prev = maze_tile;

                        maze_tile->next = maze_new;
                        maze_tile = maze_tile->next;                  
                    }
                }

                if(maze_curr->cor_x-1>=0){
                    /*  In  */
                    /*  update maze cost    */
                    /* if(BINs_arr[(maze_curr->cor_x-1)*GRID_y+maze_curr->cor_y]->R_occ > BINs_arr[(maze_curr->cor_x-1)*GRID_y+maze_curr->cor_y]->R_cap){
                        maze_ov = BINs_arr[(maze_curr->cor_x-1)*GRID_y+maze_curr->cor_y]->R_occ-BINs_arr[(maze_curr->cor_x-1)*GRID_y+maze_curr->cor_y]->R_cap;}
                    else maze_ov = 0;
                    maze_ov = 1 + var_h/(1+pow(2.71, -1*var_k*(BINs_arr[(maze_curr->cor_x-1)*GRID_y+maze_curr->cor_y]->R_occ-BINs_arr[(maze_curr->cor_x-1)*GRID_y+maze_curr->cor_y]->R_cap))); */

                    /* printf("Next : %d, Curr : %d\n", BINs_arr[(maze_curr->cor_x-1)*GRID_y+(maze_curr->cor_y)]->maze_cost+maze_ov, BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y)]->maze_cost); */
                    /* if(BINs_arr[(maze_curr->cor_x-1)*GRID_y+maze_curr->cor_y]->maze_cost+maze_ov < BINs_arr[(maze_curr->cor_x)*GRID_y+maze_curr->cor_y]->maze_cost){
                        BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y)]->maze_cost = BINs_arr[(maze_curr->cor_x-1)*GRID_y+maze_curr->cor_y]->maze_cost + maze_ov;
                    } */

                    /*  Out */
                    /*  update maze cost    */
                    maze_ov = 1 + var_h/(1+pow(2.71, -1*var_k*(BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y)]->L_occ-BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y)]->L_cap)));

                    /* printf("Next : %d, Curr : %d\n", BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y+1)]->maze_cost+maze_ov, BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y)]->maze_cost); */
                    if(BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y)]->maze_cost+maze_ov < BINs_arr[(maze_curr->cor_x-1)*GRID_y+(maze_curr->cor_y)]->maze_cost){
                        BINs_arr[(maze_curr->cor_x-1)*GRID_y+(maze_curr->cor_y)]->maze_cost = BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y)]->maze_cost+maze_ov;
                    }

                    /*  add bin */
                    if(!BINs_arr[(maze_curr->cor_x-1)*GRID_y+maze_curr->cor_y]->maze_flag){
                        BINs_arr[(maze_curr->cor_x-1)*GRID_y+maze_curr->cor_y]->maze_flag = true;

                        maze_new = malloc(sizeof(CPOINT));
                        maze_new->cor_x = maze_curr->cor_x-1;
                        maze_new->cor_y = maze_curr->cor_y;

                        /*  search from curr    */
                        /* maze_search = maze_curr;
                        while(maze_search!=NULL){
                            if(BINs_arr[maze_new->cor_x*GRID_y+(maze_new->cor_y)]->maze_cost < BINs_arr[maze_search->cor_x*GRID_y+(maze_search->cor_y)]->maze_cost){
                                maze_new->next = maze_search;
                                maze_new->prev = maze_search->prev;
                                maze_search->prev->next = maze_new;
                                maze_search->prev = maze_new;
                                break;
                            }
                            maze_search = maze_search->next;
                        }
                        if(maze_search==NULL){
                            maze_new->next = NULL;
                            maze_new->prev = maze_tile;

                            maze_tile->next = maze_new;
                            maze_tile = maze_tile->next;
                        } */
                        
                        maze_new->next = NULL;
                        maze_new->prev = maze_tile;

                        maze_tile->next = maze_new;
                        maze_tile = maze_tile->next;              
                    }
                }

                if(maze_curr->cor_x+1<GRID_x){
                    /*  In  */
                    /*  update maze cost    */
                    /* if(BINs_arr[(maze_curr->cor_x+1)*GRID_y+maze_curr->cor_y]->L_occ > BINs_arr[(maze_curr->cor_x+1)*GRID_y+maze_curr->cor_y]->L_cap){
                        maze_ov = BINs_arr[(maze_curr->cor_x+1)*GRID_y+maze_curr->cor_y]->L_occ-BINs_arr[(maze_curr->cor_x+1)*GRID_y+maze_curr->cor_y]->L_cap;}
                    else maze_ov = 0;
                    maze_ov = 1 + var_h/(1+pow(2.71, -1*var_k*(BINs_arr[(maze_curr->cor_x+1)*GRID_y+maze_curr->cor_y]->L_occ-BINs_arr[(maze_curr->cor_x+1)*GRID_y+maze_curr->cor_y]->L_cap))); */

                    /* printf("Next : %d, Curr : %d\n", BINs_arr[(maze_curr->cor_x+1)*GRID_y+(maze_curr->cor_y)]->maze_cost+maze_ov, BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y)]->maze_cost); */
                    /* if(BINs_arr[(maze_curr->cor_x+1)*GRID_y+maze_curr->cor_y]->maze_cost+maze_ov < BINs_arr[(maze_curr->cor_x)*GRID_y+maze_curr->cor_y]->maze_cost){
                        BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y)]->maze_cost = BINs_arr[(maze_curr->cor_x+1)*GRID_y+maze_curr->cor_y]->maze_cost + maze_ov;
                    } */

                    /*  Out */
                    /*  update maze cost    */
                    maze_ov = 1 + var_h/(1+pow(2.71, -1*var_k*(BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y)]->R_occ-BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y)]->R_cap)));

                    /* printf("Next : %d, Curr : %d\n", BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y+1)]->maze_cost+maze_ov, BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y)]->maze_cost); */
                    if(BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y)]->maze_cost+maze_ov < BINs_arr[(maze_curr->cor_x+1)*GRID_y+(maze_curr->cor_y)]->maze_cost){
                        BINs_arr[(maze_curr->cor_x+1)*GRID_y+(maze_curr->cor_y)]->maze_cost = BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y)]->maze_cost+maze_ov;
                    }

                    /*  add bin */
                    if(!BINs_arr[(maze_curr->cor_x+1)*GRID_y+maze_curr->cor_y]->maze_flag){
                        BINs_arr[(maze_curr->cor_x+1)*GRID_y+maze_curr->cor_y]->maze_flag = true;

                        maze_new = malloc(sizeof(CPOINT));
                        maze_new->cor_x = maze_curr->cor_x+1;
                        maze_new->cor_y = maze_curr->cor_y;

                        /*  search from curr    */
                        /* maze_search = maze_curr;
                        while(maze_search!=NULL){
                            if(BINs_arr[maze_new->cor_x*GRID_y+(maze_new->cor_y)]->maze_cost < BINs_arr[maze_search->cor_x*GRID_y+(maze_search->cor_y)]->maze_cost){
                                maze_new->next = maze_search;
                                maze_new->prev = maze_search->prev;
                                maze_search->prev->next = maze_new;
                                maze_search->prev = maze_new;
                                break;
                            }
                            maze_search = maze_search->next;
                        }
                        if(maze_search==NULL){
                            maze_new->next = NULL;
                            maze_new->prev = maze_tile;

                            maze_tile->next = maze_new;
                            maze_tile = maze_tile->next;
                        } */
                        
                        maze_new->next = NULL;
                        maze_new->prev = maze_tile;

                        maze_tile->next = maze_new;
                        maze_tile = maze_tile->next;                 
                    }
                }  
                /* printf("Bin %d, %d - Maze Cost : %d\n",maze_curr->cor_x, maze_curr->cor_y, BINs_arr[maze_curr->cor_x*GRID_y+(maze_curr->cor_y)]->maze_cost); */

                maze_curr = maze_curr->next;
            } 

            curr_x = NETs_arr[Sorted_Nets[i]]->pin_x[sink];
            curr_y = NETs_arr[Sorted_Nets[i]]->pin_y[sink];
            route_dir = 0;
            /*  Back trace  */
            /* printf("Back Trace\n"); */
            while(curr_x!=NETs_arr[Sorted_Nets[i]]->pin_x[source] || curr_y!=NETs_arr[Sorted_Nets[i]]->pin_y[source]){
                /*  printf("Search Bin : %d %d\n", curr_x, curr_y); */
                /* printf("distance cost\n"); */
                /*  distance cost   */
                manh[0] = abs(curr_x-NETs_arr[Sorted_Nets[i]]->pin_x[source]) + abs((curr_y+1)-NETs_arr[Sorted_Nets[i]]->pin_y[source]);
                manh[1] = abs(curr_x-NETs_arr[Sorted_Nets[i]]->pin_x[source]) + abs((curr_y-1)-NETs_arr[Sorted_Nets[i]]->pin_y[source]);
                manh[2] = abs((curr_x-1)-NETs_arr[Sorted_Nets[i]]->pin_x[source]) + abs(curr_y-NETs_arr[Sorted_Nets[i]]->pin_y[source]);
                manh[3] = abs((curr_x+1)-NETs_arr[Sorted_Nets[i]]->pin_x[source]) + abs(curr_y-NETs_arr[Sorted_Nets[i]]->pin_y[source]);

                /*  Maze Cost   */
                /* printf("Maze Cost\n"); */
                if(curr_y+1<GRID_y){
                    if(BINs_arr[curr_x*GRID_y+curr_y+1]->maze_cost<0) cong[0] = ROUTED_PENALTY;
                    else cong[0] = BINs_arr[curr_x*GRID_y+curr_y+1]->maze_cost + 1 + var_h/(1+pow(2.71, -1*var_k*(BINs_arr[curr_x*GRID_y+(curr_y+1)]->B_occ-BINs_arr[curr_x*GRID_y+(curr_y+1)]->B_cap)));}
                else cong[0] = BOUNDARY_PENALTY;

                if(curr_y-1>=0){
                    if(BINs_arr[curr_x*GRID_y+curr_y-1]->maze_cost<0) cong[1] = ROUTED_PENALTY;
                    else cong[1] = BINs_arr[curr_x*GRID_y+curr_y-1]->maze_cost + 1 + var_h/(1+pow(2.71, -1*var_k*(BINs_arr[curr_x*GRID_y+(curr_y-1)]->T_occ-BINs_arr[curr_x*GRID_y+(curr_y-1)]->T_cap)));}
                else cong[1] = BOUNDARY_PENALTY;

                if(curr_x-1>=0){
                    if(BINs_arr[(curr_x-1)*GRID_y+curr_y]->maze_cost<0) cong[2] = ROUTED_PENALTY;
                    else cong[2] = BINs_arr[(curr_x-1)*GRID_y+curr_y]->maze_cost + 1 + var_h/(1+pow(2.71, -1*var_k*(BINs_arr[(curr_x-1)*GRID_y+curr_y]->R_occ-BINs_arr[(curr_x-1)*GRID_y+curr_y]->R_cap)));}
                else cong[2] = BOUNDARY_PENALTY;

                if(curr_x+1<GRID_x){
                    if(BINs_arr[(curr_x+1)*GRID_y+curr_y]->maze_cost<0) cong[3] = ROUTED_PENALTY;
                    else cong[3] = BINs_arr[(curr_x+1)*GRID_y+curr_y]->maze_cost + 1 + var_h/(1+pow(2.71, -1*var_k*(BINs_arr[(curr_x+1)*GRID_y+curr_y]->L_occ-BINs_arr[(curr_x+1)*GRID_y+curr_y]->L_cap)));}
                else cong[3] = BOUNDARY_PENALTY;

                /*  Total Cost    */
                /*  don't find previous bin */ 
                /* printf("Old Cost\n"); */
                cost[0] = cong[0] + 0.01*manh[0]/(GRID_x+GRID_y);
                cost[1] = cong[1] + 0.01*manh[1]/(GRID_x+GRID_y);
                cost[2] = cong[2] + 0.01*manh[2]/(GRID_x+GRID_y);
                cost[3] = cong[3] + 0.01*manh[3]/(GRID_x+GRID_y);
                
                /* printf("find min cost\n"); */
                /*  find min cost   */
                for(j=0; j<4; ++j){
                    if(cost[j] <= cost[route_dir]) route_dir = j;
                }                

                /* printf("route\n"); */
                /*  route   */
                /*  update curr_x, curr_y   */
                /*  update occupy   */
                switch(route_dir){
                    case 0 :
                        curr_y++;
                        break;
                    case 1 :
                        curr_y--;
                        break;
                    case 2 :
                        curr_x--; 
                        break;
                    case 3 :
                        curr_x++; 
                        break;
                    default : break;
                }

                BINs_arr[curr_x*GRID_y+curr_y]->maze_cost = -1*(route_dir+1);
            }        

            curr_x = NETs_arr[Sorted_Nets[i]]->pin_x[source];
            curr_y = NETs_arr[Sorted_Nets[i]]->pin_y[source];
            route_dir = 0;
            /* printf("Reroute\n"); */
            /*  Reroute */
            while(curr_x!=NETs_arr[Sorted_Nets[i]]->pin_x[sink] || curr_y!=NETs_arr[Sorted_Nets[i]]->pin_y[sink]){
               /*  printf("Search Bin : %d %d\n", curr_x, curr_y); */
                /*  congestion cost */
                /*  current penalty */                
                if(BINs_arr[curr_x*GRID_y+curr_y]->T_occ >= BINs_arr[curr_x*GRID_y+curr_y]->T_cap){
                    cong[0] = BINs_arr[curr_x*GRID_y+curr_y]->T_occ - BINs_arr[curr_x*GRID_y+curr_y]->T_cap + BINs_arr[curr_x*GRID_y+curr_y]->T_hp;
                }
                else cong[0] = BINs_arr[curr_x*GRID_y+curr_y]->T_hp;

                if(BINs_arr[curr_x*GRID_y+curr_y]->B_occ >= BINs_arr[curr_x*GRID_y+curr_y]->B_cap){
                    cong[1] = BINs_arr[curr_x*GRID_y+curr_y]->B_occ - BINs_arr[curr_x*GRID_y+curr_y]->B_cap + BINs_arr[curr_x*GRID_y+curr_y]->B_hp;
                }
                else cong[1] = BINs_arr[curr_x*GRID_y+curr_y]->B_hp;

                if(BINs_arr[curr_x*GRID_y+curr_y]->L_occ >= BINs_arr[curr_x*GRID_y+curr_y]->L_cap){
                    cong[2] = BINs_arr[curr_x*GRID_y+curr_y]->L_occ - BINs_arr[curr_x*GRID_y+curr_y]->L_cap + BINs_arr[curr_x*GRID_y+curr_y]->L_hp;
                }
                else cong[2] = BINs_arr[curr_x*GRID_y+curr_y]->L_hp;

                if(BINs_arr[curr_x*GRID_y+curr_y]->R_occ >= BINs_arr[curr_x*GRID_y+curr_y]->R_cap){
                    cong[3] = BINs_arr[curr_x*GRID_y+curr_y]->R_occ - BINs_arr[curr_x*GRID_y+curr_y]->R_cap + BINs_arr[curr_x*GRID_y+curr_y]->R_hp;
                }
                else cong[3] = BINs_arr[curr_x*GRID_y+curr_y]->R_hp;

                /* printf("distance cost\n"); */
                /*  distance cost   */
                manh[0] = abs(curr_x-NETs_arr[Sorted_Nets[i]]->pin_x[sink]) + abs((curr_y+1)-NETs_arr[Sorted_Nets[i]]->pin_y[sink]);
                manh[1] = abs(curr_x-NETs_arr[Sorted_Nets[i]]->pin_x[sink]) + abs((curr_y-1)-NETs_arr[Sorted_Nets[i]]->pin_y[sink]);
                manh[2] = abs((curr_x-1)-NETs_arr[Sorted_Nets[i]]->pin_x[sink]) + abs(curr_y-NETs_arr[Sorted_Nets[i]]->pin_y[sink]);
                manh[3] = abs((curr_x+1)-NETs_arr[Sorted_Nets[i]]->pin_x[sink]) + abs(curr_y-NETs_arr[Sorted_Nets[i]]->pin_y[sink]); 

                /*  space cost  */
                space[0] = GRID_x;
                space[1] = GRID_x;
                space[2] = GRID_y;
                space[3] = GRID_y;

                /*  Maze Cost   */
                /* printf("Maze Cost\n"); */
                if(curr_y+1<GRID_y) cong[0] = (10000/MAZE_INF)*BINs_arr[curr_x*GRID_y+curr_y+1]->maze_cost;
                else cong[0] = MAZE_INF;

                if(curr_y-1>=0) cong[1] = (10000/MAZE_INF)*BINs_arr[curr_x*GRID_y+curr_y-1]->maze_cost;
                else cong[1] = MAZE_INF;

                if(curr_x-1>=0) cong[2] = (10000/MAZE_INF)*BINs_arr[(curr_x-1)*GRID_y+curr_y]->maze_cost;
                else cong[2] = MAZE_INF;

                if(curr_x+1<GRID_x) cong[3] = (10000/MAZE_INF)*BINs_arr[(curr_x+1)*GRID_y+curr_y]->maze_cost;
                else cong[3] = MAZE_INF;
                
                /*  Total Cost    */
                /*  don't find previous bin */ 
                /* printf("Old Cost\n"); */
                if(curr_y+1<GRID_y){
                    cost[0] = (1-alpha)*((1-beta)*(cong[0]) + beta*space[0]) + alpha*manh[0] + BINs_arr[curr_x*GRID_y+(curr_y+1)]->routed*ROUTED_PENALTY;
                }
                else cost[0] = (1-alpha)*((1-beta)*(cong[0]) + beta*space[0]) + alpha*manh[0] + BOUNDARY_PENALTY;

                if(curr_y-1>=0){
                    cost[1] = (1-alpha)*((1-beta)*(cong[1]) + beta*space[1]) + alpha*manh[1] + BINs_arr[curr_x*GRID_y+(curr_y-1)]->routed*ROUTED_PENALTY;
                }
                else cost[1] = (1-alpha)*((1-beta)*(cong[1]) + beta*space[1]) + alpha*manh[1] + BOUNDARY_PENALTY;

                if(curr_x-1>=0){
                    cost[2] = (1-alpha)*((1-beta)*(cong[2]) + beta*space[2]) + alpha*manh[2] + BINs_arr[(curr_x-1)*GRID_y+curr_y]->routed*ROUTED_PENALTY;
                }
                else cost[2] = (1-alpha)*((1-beta)*(cong[2]) + beta*space[2]) + alpha*manh[2] + BOUNDARY_PENALTY;

                if(curr_x+1<GRID_x){
                    cost[3] = (1-alpha)*((1-beta)*(cong[3]) + beta*space[3]) + alpha*manh[3] + BINs_arr[(curr_x+1)*GRID_y+curr_y]->routed*ROUTED_PENALTY;
                }
                else cost[3] = (1-alpha)*((1-beta)*(cong[3]) + beta*space[3]) + alpha*manh[3] + BOUNDARY_PENALTY;
                
                /* printf("find min cost\n"); */
                /*  find min cost   */
                for(j=0; j<4; ++j){
                    if(cost[j] <= cost[route_dir]) route_dir = j;
                }

                switch(BINs_arr[curr_x*GRID_y+curr_y]->maze_cost){
                    case -1 :
                        route_dir = 1;
                        break;
                    case -2 :
                        route_dir = 0;
                        break;
                    case -3 :
                        route_dir = 3;
                        break;
                    case -4 :
                        route_dir = 2;
                        break;
                    default :
                        break;
                }

                /* printf("route\n"); */
                /*  route   */
                /*  update curr_x, curr_y   */
                /*  update occupy   */
                switch(route_dir){
                    case 0 :
                        BINs_arr[curr_x*GRID_y+curr_y]->T_occ++;
                        BINs_arr[curr_x*GRID_y+(curr_y+1)]->B_occ++;
                        curr_y++;
                        break;
                    case 1 :
                        BINs_arr[curr_x*GRID_y+curr_y]->B_occ++;
                        BINs_arr[curr_x*GRID_y+(curr_y-1)]->T_occ++;
                        curr_y--;
                        break;
                    case 2 :
                        BINs_arr[curr_x*GRID_y+curr_y]->L_occ++;
                        BINs_arr[(curr_x-1)*GRID_y+curr_y]->R_occ++;
                        curr_x--; 
                        break;
                    case 3 :
                        BINs_arr[curr_x*GRID_y+curr_y]->R_occ++; 
                        BINs_arr[(curr_x+1)*GRID_y+curr_y]->L_occ++;
                        curr_x++; 
                        break;
                    default : break;
                }

                if(curr_point->prev!=NULL){
                    if(curr_point->prev->cor_x==curr_x || curr_point->prev->cor_y==curr_y){
                        if(curr_point->cor_x==curr_x) curr_point->cor_y = curr_y;
                        if(curr_point->cor_y==curr_y) curr_point->cor_x = curr_x;
                    }
                    else{
                        /*  new point   */
                        new_point = malloc(sizeof(CPOINT));
                        new_point->cor_x = curr_x;
                        new_point->cor_y = curr_y;
                                
                        curr_point->next = new_point;
                        new_point->prev = curr_point;
                        new_point->next = NULL;
                        curr_point = new_point;
                    }                    
                }
                else{
                    /*  new point   */
                    new_point = malloc(sizeof(CPOINT));
                    new_point->cor_x = curr_x;
                    new_point->cor_y = curr_y;
                                
                    curr_point->next = new_point;
                    new_point->prev = curr_point;
                    new_point->next = NULL;
                    curr_point = new_point;
                } 

                BINs_arr[curr_x*GRID_y+curr_y]->routed = true;
                /* if(alpha < alpha_cap) alpha += step; */
            }
            
            for(k=0; k<GRID_x; ++k){
                for(j=0; j<GRID_y; ++j){
                    BINs_arr[k*GRID_y+j]->routed = false;
                    BINs_arr[k*GRID_y+j]->maze_cost = MAZE_INF;
                    BINs_arr[k*GRID_y+j]->maze_flag = false;
                }
            }            
        }

        end = clock();
        printf("Time(Route) : %0.1f\n", (double)(end - begin) / CLOCKS_PER_SEC);

        overflow = 0;        
        for(i=0; i<GRID_x; ++i){
            for(j=0; j<GRID_y; ++j){
                /*  update hp   */
                /*  calculate total overflow    */
                if(BINs_arr[i*GRID_y+j]->T_occ > BINs_arr[i*GRID_y+j]->T_cap){
                    if(BINs_arr[i*GRID_y+j]->T_cap!=BOUNDARY_CAP){
                        overflow += (BINs_arr[i*GRID_y+j]->T_occ-BINs_arr[i*GRID_y+j]->T_cap);
                        BINs_arr[i*GRID_y+j]->T_hp+=1;
                    }                    
                }
                else BINs_arr[i*GRID_y+j]->T_hp = 1;

                if(BINs_arr[i*GRID_y+j]->B_occ > BINs_arr[i*GRID_y+j]->B_cap){
                    if(BINs_arr[i*GRID_y+j]->B_cap!=BOUNDARY_CAP){
                        overflow += (BINs_arr[i*GRID_y+j]->B_occ-BINs_arr[i*GRID_y+j]->B_cap);
                        BINs_arr[i*GRID_y+j]->B_hp+=1;
                    }                    
                }
                else BINs_arr[i*GRID_y+j]->B_hp = 1;

                if(BINs_arr[i*GRID_y+j]->L_occ > BINs_arr[i*GRID_y+j]->L_cap){
                    if(BINs_arr[i*GRID_y+j]->L_cap!=BOUNDARY_CAP){
                        overflow += (BINs_arr[i*GRID_y+j]->L_occ-BINs_arr[i*GRID_y+j]->L_cap);
                        BINs_arr[i*GRID_y+j]->L_hp+=1;
                    }                    
                }
                else BINs_arr[i*GRID_y+j]->L_hp = 1;

                if(BINs_arr[i*GRID_y+j]->R_occ > BINs_arr[i*GRID_y+j]->R_cap){
                    if(BINs_arr[i*GRID_y+j]->R_cap!=BOUNDARY_CAP){
                        overflow += (BINs_arr[i*GRID_y+j]->R_occ-BINs_arr[i*GRID_y+j]->R_cap);
                        BINs_arr[i*GRID_y+j]->R_hp+=1;
                    }                    
                }
                else BINs_arr[i*GRID_y+j]->R_hp = 1;
            }
        }
        overflow = overflow/2;
        printf("Overflow = %d\n", overflow);

        /*  if total wirelenth or total overflow update < 10% then break   */
        if(total_overflow-overflow<=total_overflow*terminate_ratio && total_overflow!=0) break;
        else total_overflow = overflow;

        if(iteration < cnt) break;
        else cnt++;
    }

    route_end = clock();
    printf("Time(Total Route) : %0.1f\n", (double)(route_end - route_begin) / CLOCKS_PER_SEC);    

    begin = clock();

    /*  output  */
    for(i=0; i<NUM_NETs; ++i){
        fprintf(out_file, "net%d %d\n", NETs_arr[i]->net_id, NETs_arr[i]->net_id);
        curr_point = NETs_arr[i]->route;
        while(curr_point!=NULL){
            if(curr_point->next!=NULL){
                fprintf(out_file, "(%d, %d, 1)-(%d, %d, 1)\n", curr_point->cor_x, curr_point->cor_y, curr_point->next->cor_x, curr_point->next->cor_y);                
            }          
            curr_point = curr_point->next;  
        }
        fprintf(out_file, "!\n");
    }

    end = clock();
    printf("Time(Output file) : %0.1f\n", (double)(end - begin) / CLOCKS_PER_SEC);

    fclose(nets_file);
    fclose(out_file);

    total_end = clock();
    printf("Time(Total) : %0.1f\n", (double)(total_end - total_begin) / CLOCKS_PER_SEC);

    return 0;
}

FILE *R_file(char *file, char *mode){
	FILE *f;
	
	f = fopen(file, mode);
	if(f == NULL){
		printf("Unable to open %s.\n", file);		
		return NULL;
	}

	printf("Open %s successfully.\n", file);
	return f;
}