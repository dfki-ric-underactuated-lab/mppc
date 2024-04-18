import copy
import csv

class Storage:
    def __init__(self, path):
        self.parkour_list = []
        self.result_list = []
        self.parkour_path = "/".join([path, "parkourData.csv"])
        self.result_path = "/".join([path, "resultData.csv"])

    def store(self, Parkour, res_dict):
        self.parkour_list.append(copy.deepcopy(Parkour))
        self.result_list.append(copy.deepcopy(res_dict))

    def save(self):
        with open(self.parkour_path, 'w', newline='') as csvfile:
            fieldnames_p = self.parkour_list[0].keys()
            writer_p = csv.DictWriter(csvfile, fieldnames=fieldnames_p)
            writer_p.writeheader()
            for park in self.parkour_list:
                writer_p.writerow(park)

        with open(self.result_path, 'w', newline='') as csvfile:
            fieldnames_r = self.result_list[0].keys()
            writer_r = csv.DictWriter(csvfile, fieldnames=fieldnames_r)
            writer_r.writeheader()
            for res in self.result_list:
                writer_r.writerow(res)
