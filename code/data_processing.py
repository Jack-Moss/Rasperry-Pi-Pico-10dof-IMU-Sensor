from math import isclose

""""Climber falls off. U = 0, ActualAcc = 9.81
(1) Free fall (rope is slack). DeviceAcc = 0. Actual Acc = 9.81 - DeviceAcc
(2) Rope starts to stretch. DeviceAcc > 0. Actual Acc = 9.81 - DeviceAcc
(3) Rope force equals gravity. DeviceAcc = 9.81, Actual Acc = 9.81 - DeviceAcc = 0
The acceleration at this point is zero, but the climbers velocity is high
(4) Rope continues to stretch due to the climbers kinetic energy. 
The DeviceAcc is now greater than 9.81, and the ActualAcc is now negative 
(climber is slowing down), but the distance continues to increase, and the velocity is still positive.
DeviceAcc > 9.81, ActualAcc = 9.81 - DeviceAcc, which is negative.
(5) The greatest distance is reached when the velocity reaches zero. 
The negative acceleration will be at its most negative at this point."""


def run_calculations(dynamic_list, climber_weight):
    #print("Dynamic List")
    #print(dynamic_list)
    cleaned_list = sanitise_list(dynamic_list)
    #print("Cleaned List")
    #print(cleaned_list)
    trimmed_list = create_trim_list(cleaned_list)
    #print("Trimmed List")
    #print(trimmed_list)
    max_deceleration = find_max_deceleration(trimmed_list)
    peak_kn = find_peak_kn(max_deceleration, climber_weight)
    fall_distance = find_total_distance(trimmed_list)
    return {"max deceleration": max_deceleration, "peak kn": peak_kn, "fall distance": fall_distance, "trimmed and sanitised fall data": trimmed_list}


def sanitise_list(data_list):
    """:param data_list : list[[microsecond timestamp, acceleration]]
        This function iterates through the initial list and subtracts the initial timestamp from each value
        then divides each value by 1000000 to convert microseconds to seconds
    """
    unclean = data_list
    first_value = unclean[0][0]
    for i in unclean:
        i[0] -= first_value
        i[0] = i[0] / 1000000
    return unclean


def create_trim_list(data_list):
    """This iterates through the list, and with the use of three flag variables first searches for the start of the fall
    which is defined as the first three variables that are below 9.81ms and then saves that as the start index.

    The function continues through the list until it finds the next three variables that are approximately 9.81ms, this
    is then saved as the end index.

    The list is then trimmed and returned.

    :param data_list: List[]
    :return: trimmed_list: List[]
    """
    #print("Data list handed to trim")
    #print(data_list)
    flag = 0
    fall_start_index = 0
    fall_end_index = 0
    for count, item in enumerate(data_list):
        if fall_start_index == 0:
            if item[1] < 9 and flag != 3:
                flag += 1
            elif flag == 3:
                fall_start_index = count - 10
                flag = 0
            else:
                flag = 0

        if fall_end_index == 0 and fall_start_index != 0:
            if flag == 3:
                fall_end_index = count
            elif isclose(item[1], 9.81, abs_tol=0.5):
                flag += 1
            else:
                flag = 0

    trimmed_list = data_list[fall_start_index:fall_end_index]
    return trimmed_list


def find_max_deceleration(data_list):
    """:param data_list : list[[microsecond timestamp, acceleration]...]"""
    max_deceleration = 0
    for i in data_list:
        if i[1] > max_deceleration:
            max_deceleration = i[1]
    return max_deceleration


def find_total_distance(data_list):
    """This needs to work differently, probably needs to work by the start of the fall to the end, cos fall length can
    probably be estimated since g is consistent?

    free-fall might be cool to know?

    :param data_list : list[[microsecond timestamp, acceleration]...]"""
    total_distance_fallen = 0
    total_fall_duration = 0
    velocity = 0
    previous_time = 0
    # reading = False
    # need to cut out the section only of the falling moments, and the second that acceleration goes back to above 9.81
    # it views it as static
    for i in data_list:
        # if acceleration goes under 9.81 then set reading fall to true until reading is above 9.81
        time = i[0] - previous_time
        previous_time = i[0]
        total_fall_duration += time
        acceleration = 9.81 - i[1]
        distance_in_this_frame = velocity * time + 0.5 * acceleration * time ** 2
        total_distance_fallen += distance_in_this_frame
        velocity = velocity + acceleration * time

    return "Meters = ", total_distance_fallen, "Seconds = ", total_fall_duration


def find_peak_kn(max_deceleration, climber_weight):
    force = max_deceleration * climber_weight
    return force / 1000


def data_test():
    test_dict ={"metadata": {"max deceleration": 0, "fall distance": ["Meters = ", 0, "Seconds = ", 0], "trimmed and sanitised fall data": [], "peak kn": 0.0}, "raw fall data": [[0.0, 10.13354], [0.009836, 10.31627], [0.019677, 10.83142], [0.029524, 11.09178], [0.039365, 11.47425], [0.049214, 11.54311], [0.059048, 11.51759], [0.068888, 11.27055], [0.078729, 11.1184], [0.088578, 10.9755], [0.09841001, 10.78264], [0.108256, 10.83298], [0.118099, 11.04186], [0.127947, 11.10869], [0.137789, 11.13691], [0.147631, 11.09993], [0.157464, 11.07864], [0.16732, 11.07261], [0.177167, 11.10477], [0.187008, 11.14909], [0.196838, 11.24658], [0.206689, 11.27158], [0.216522, 11.22235], [0.226373, 11.14252], [0.236216, 11.05367], [0.246049, 10.91718], [0.255882, 10.88757], [0.265738, 10.89411], [0.27558, 10.9218], [0.285426, 10.95919], [0.295264, 10.84482], [0.30511, 10.73151], [0.31496, 10.61085], [0.32481, 10.60816], [0.334655, 10.65105], [0.344495, 10.77459], [0.354335, 10.80906], [0.364173, 10.77289], [0.374001, 10.69447], [0.383846, 10.55418], [0.393676, 10.26904], [0.403506, 10.16755], [0.413343, 10.05178], [0.423188, 10.03171], [0.433032, 10.036], [0.442869, 10.01549], [0.452707, 9.996376], [0.462564, 10.04521], [0.47241, 10.10307], [0.482263, 10.19839], [0.49209, 10.46006], [0.501939, 10.58691], [0.511782, 10.84995], [0.521635, 10.95691], [0.531456, 11.0323], [0.541298, 10.98217], [0.55113, 10.87554], [0.560981, 10.67752], [0.5708241, 10.62507], [0.580672, 10.62521], [0.59053, 10.74543], [0.600369, 10.81224], [0.610203, 10.91132], [0.620046, 10.90855], [0.629906, 10.84452], [0.639744, 10.65949], [0.649589, 10.57048], [0.659433, 10.5122], [0.669283, 10.53865], [0.679107, 10.56121], [0.688943, 10.58693], [0.698792, 10.57446], [0.708636, 10.5047], [0.71848, 10.44332], [0.728313, 10.3839], [0.738166, 10.37081], [0.748002, 10.39848], [0.757842, 10.44496], [0.767682, 10.51473], [0.77753, 10.49601], [0.787362, 10.341], [0.797208, 10.22683], [0.807048, 10.09748], [0.8168961, 9.821103], [0.826735, 9.659936], [0.836572, 9.403837], [0.846412, 9.293316], [0.856251, 9.152677], [0.866085, 8.562891], [0.87593, 7.055573], [0.885765, 4.565056], [0.895623, 3.706626], [0.9054621, 3.045851], [0.915304, 2.137929], [0.925138, 1.826211], [0.934978, 1.407263], [0.944821, 1.3121], [0.95466, 1.194252], [0.9645041, 1.041453], [0.97435, 0.9926789], [0.984266, 0.9344129], [0.99413, 0.9190311], [1.00398, 0.9132935], [1.013846, 0.9162055], [1.023715, 0.9144852], [1.033594, 0.9217068], [1.043466, 0.9313161], [1.053323, 0.9498072], [1.063181, 0.9733728], [1.073042, 0.9921183], [1.082911, 1.026511], [1.092782, 1.047111], [1.102642, 1.0712], [1.112504, 1.101077], [1.122365, 1.116729], [1.13225, 1.138651], [1.142107, 1.15787], [1.151965, 1.173158], [1.161838, 1.213605], [1.171701, 1.227978], [1.181578, 1.261628], [1.19143, 1.291271], [1.201294, 1.314506], [1.211149, 1.371827], [1.220999, 1.400635], [1.230872, 1.443232], [1.240724, 1.46795], [1.250568, 1.487889], [1.260421, 1.53674], [1.270274, 1.561502], [1.28014, 1.604912], [1.289996, 1.634494], [1.299855, 1.664076], [1.309705, 1.717772], [1.31956, 1.741898], [1.329434, 1.804651], [1.339295, 1.829424], [1.349155, 1.853048], [1.359031, 1.911079], [1.368891, 1.939123], [1.378771, 1.972543], [1.38865, 1.991895], [1.398517, 2.016], [1.408359, 2.059068], [1.418216, 2.097729], [1.428079, 9.212284], [1.437915, 61.87501], [1.447769, 104.4281], [1.457641, 114.6524], [1.467501, 105.0813], [1.477375, 78.70969], [1.487235, 65.83469], [1.497089, 54.32601], [1.50695, 35.63296], [1.516819, 28.53199], [1.526688, 18.15779], [1.536548, 14.40569], [1.546402, 11.39239], [1.556263, 7.129955], [1.566128, 5.693942], [1.575986, 3.704936], [1.585858, 3.032454], [1.595706, 2.524376], [1.605571, 1.812545], [1.615436, 1.556946], [1.625307, 1.50922], [1.635154, 2.114117], [1.645006, 3.196158], [1.654878, 5.970157], [1.664718, 8.419822], [1.67459, 12.10114], [1.684453, 13.60507], [1.694303, 14.59847], [1.704155, 15.21238], [1.714015, 14.8772], [1.723881, 13.42028], [1.73373, 12.50092], [1.743577, 11.54026], [1.753435, 9.833858], [1.763294, 9.255918], [1.773149, 9.007174], [1.783003, 9.331883], [1.792848, 9.839613], [1.802717, 11.13182], [1.812571, 11.89175], [1.822436, 13.03431], [1.832283, 13.01041], [1.842118, 12.6302], [1.851985, 11.46302], [1.861833, 10.93385], [1.871691, 10.18017], [1.881557, 9.962128], [1.891407, 9.859559], [1.901258, 9.986195], [1.911117, 10.14833], [1.920993, 10.50687], [1.930847, 10.61039], [1.9407, 10.67977], [1.950571, 10.72839], [1.960434, 10.72437], [1.970294, 10.70252], [1.980157, 10.68669], [1.990005, 10.6591], [1.999866, 10.58517], [2.009719, 10.54528], [2.019578, 10.4916], [2.02943, 10.49102], [2.039285, 10.50515], [2.049149, 10.55793], [2.059002, 10.58794], [2.068875, 10.59377], [2.078747, 10.58919], [2.08861, 10.59022], [2.098476, 10.56011], [2.108337, 10.52842], [2.118203, 10.51879], [2.128055, 10.54154], [2.137908, 10.56559], [2.147772, 10.58798], [2.157632, 10.57673], [2.167498, 10.55112], [2.177367, 10.54361], [2.187208, 10.5364], [2.197079, 10.53973], [2.206929, 10.55068], [2.216791, 10.57378], [2.226655, 10.57276], [2.236509, 10.55796], [2.246373, 10.54104], [2.256232, 10.54705], [2.26611, 10.56639], [2.275971, 10.56283], [2.285824, 10.55782], [2.295681, 10.55851], [2.305531, 10.55146], [2.315395, 10.53883], [2.325254, 10.54631], [2.335105, 10.55552], [2.344977, 10.55867], [2.354834, 10.55965], [2.364705, 10.55217], [2.374569, 10.54472], [2.384446, 10.54472], [2.394309, 10.54757], [2.404159, 10.55116], [2.414028, 10.55576], [2.423878, 10.55046], [2.433726, 10.54198], [2.443591, 10.52288], [2.453445, 10.52288], [2.463321, 10.55894], [2.47318, 10.57272], [2.483034, 10.56957], [2.492902, 10.50123], [2.502766, 10.50369], [2.512642, 10.63674], [2.522495, 10.6796], [2.532359, 10.6778], [2.542226, 10.49407], [2.5521, 10.41952], [2.561964, 10.51991], [2.571834, 10.6134], [2.581683, 10.64762], [2.591544, 10.57313], [2.601386, 10.53782], [2.611246, 10.53523], [2.621109, 10.54665], [2.630962, 10.5362], [2.64083, 10.52008], [2.650691, 10.5697], [2.660573, 10.63605], [2.670433, 10.6019], [2.680295, 10.53912], [2.690168, 10.49052], [2.712451, 10.59906], [2.722338, 10.57961], [2.732194, 10.55172], [2.742059, 10.54466], [2.751914, 10.55187], [2.761768, 10.55077], [2.771647, 10.55217], [2.781505, 10.55035], [2.791353, 10.54432], [2.801217, 10.54656], [2.81107, 10.5644], [2.820937, 10.56301], [2.830794, 10.54523], [2.840647, 10.54234], [2.8505, 10.55851], [2.860355, 10.56168], [2.87024, 10.55709], [2.880109, 10.54054], [2.889969, 10.55475], [2.899846, 10.58452], [2.909704, 10.57638], [2.919581, 10.55437], [2.929454, 10.54269], [2.939312, 10.55332], [2.949185, 10.55146], [2.959039, 10.54788]]}
    test_list = test_dict.get("raw fall data")
    for time, speed in test_list:
        print(str(time) + "," + str(speed))
        print(create_trim_list(test_list))
    for time, speed in create_trim_list(test_list):
        print(str(time) + "," + str(speed))


def estimate_fall_duration(self):
    pass


def estimate_fall_length(self):
    for entry in self.fall_data:
        print(entry)
    # time from around 5ms to back above 5ms?
