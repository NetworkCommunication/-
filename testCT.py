import Get_Move as Gm
import Init
import numpy as np
import Global_Par as Gp
import time as t
import jhmmtg as jh
import junction_init as ji
import DelayCalculate as dc
import EXP_ABS as Ea
import math

def Tij_calculate(controller, node_list, time):
    G = controller.junction_matrix
    for u in G.nodes():
        for u, v in G.edges(u):  # u's evaluation of v, v's distance change rate relative to u, speed change rate, connection duration

            distance_uv = Ea.distance(node_list[u].position, node_list[v].position)
            if 0 < distance_uv < Gp.com_dis and u != v:
                # Judging the distance
                lat_distance = controller.feature_junction_matrix[u][v]['lat_dist_uv']
                # distance becomes farther
                if distance_uv > lat_distance and lat_distance != 0:
                    G[u][v]['score'] = (math.sqrt(
                        math.pow(Gp.com_dis, 2) - math.pow(node_list[u].position[0] - node_list[v].position[0],
                                                           2)) - math.fabs(
                        node_list[u].position[1] - node_list[v].position[1])) / Ea.distance(node_list[u].velocity,
                                                                                            node_list[v].velocity)
                else:
                    G[u][v]['score'] = (math.sqrt(
                        math.pow(Gp.com_dis, 2) + math.pow(node_list[u].position[0] - node_list[v].position[0],
                                                           2)) - math.fabs(
                        node_list[u].position[1] - node_list[v].position[1])) / Ea.distance(node_list[u].velocity,
                                                                                            node_list[v].velocity)
    return
def start(file_name, max_delay):
    node_list = []
    # com_node_list = []

    sim_time = 309  # int(input("sim_time:"))
    # location file read
    try:
        movement_matrix, init_position_matrix = Gm.get_position(file_name)
    except Exception:
        movement_matrix, init_position_matrix = Gm.get_position_X_(file_name)
    node_num = init_position_matrix.shape[0]
    # Controller initialization
    controller = Init.init_controller(node_num, 'DiG')
    controller.feature_junction_matrix_construction(node_num)
    # Location data processing
    init_position_arranged = init_position_matrix[np.lexsort(init_position_matrix[:, ::-1].T)]

    node_position = init_position_arranged[0]
    ji.inti()

    # Node initialization
    node_list = (Init.init_node(node_position, controller))
    effi = 0
    delay = 0
    std2 = 0
    pt_sum = 0
    round = 10
    # Generate communication nodes
    for i in range(round):
        com_node_list = []
        com_node_list.extend(Init.get_communication_node(node_num-1))
        start_time = t.time()

        # in seconds
        for time in range(200, sim_time):
            print('\nTime: %d' % time)
            # Process position matrix
            current_move = movement_matrix[np.nonzero(movement_matrix[:, 0].A == time)[0], :]
            for value in current_move:
                for i in range(1, 4):
                    node_position[int(value[0, 1]), i] = value[0, i+1]
            node_id_position = node_position[:, [1, 2, 3]]
            # All nodes update location and send hello to controller
            for node in node_list:
                node.update_node_position(node_id_position)
                node.generate_hello(controller)
            jh.num_count()
            # Controller updates network global situation
            controller.predict_position()
            controller.junction_matrix_construction(node_num)
            pt_s = t.time()
            Tij_calculate(controller, node_list, time)
            pt_e = t.time()
            pt_sum += pt_e - pt_s
            # All communication nodes generate packets and send requests to the controller
            node_list[com_node_list[time % int((node_num * Gp.com_node_rate) / 2 - 1)][0]].generate_request(
                com_node_list[time % int((node_num * Gp.com_node_rate) / 2 - 1)][1], controller, 1024)
            # Controller handles routing requests
            print('\nrouting request')
            controller.resolve_request(node_list, 2)

            # All nodes handle misrouting repair requests
            print('\nerror request')
            controller.resolve_error(node_list, 2)
            print('\nforward')

            # All nodes start forwarding packets
            for node in node_list:
                node.forward_pkt_to_nbr(node_list, controller)
            jh.delete()

        end_time = t.time()
        effi += end_time-start_time
        delay += Gp.sum
        Gp.sum = 0
        pt_sum = 0
        std2 += np.std(Gp.record, ddof=1)
        Gp.record.clear()
    with open('result/data_testLinkduration', 'a', encoding='utf-8') as f:
        f.write('\ncalculation time:\n')
        f.write(str(effi / round))
        f.write('\ndelay:\n')
        f.write(str(dc.delay_cal_dijk(max_delay) / (Gp.success_route + Gp.fail_route) / round))
        f.write('\ndelivery ratio:\n')
        f.write('{},{},{},{},{}'.format(Gp.fail_route, Gp.success_route, Gp.success_times, Gp.fail_times,
                                        Gp.success_times / (Gp.success_times + Gp.fail_times)))
        f.write('\njitter:\n')
        f.write(str(std2 / Gp.pps / round))
    f.close()
if __name__ == '__main__':
    MAX_DELAY = [61.96704364923099, 899.5857541647147, 899.5857541647147, 4082.201330988286, 4082.201330988286]
    for i in range(5):
        file_name = 'data/3000-1500/{}mobility.tcl'.format(i*100 + 200)
        with open('result/data_testLinkduration', 'a', encoding='utf-8') as f:
            f.write('\n'+'-'*30+'\n')
            f.write(file_name)
        f.close()
        print(file_name)
        start(file_name, MAX_DELAY[i])
