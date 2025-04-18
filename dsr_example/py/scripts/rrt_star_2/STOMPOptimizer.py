import numpy as np
import rospy
class StompOptimizer:
    def __init__(self, robot, initial_path, obstacle_list, num_iterations=100, step_size=0.1, noise_std_dev=0.05):
        self.robot = robot
        self.path = np.array(initial_path)  # path: N x D (N=numero di punti, D=gradi di libertà)
        self.obstacle_list = obstacle_list
        self.num_iterations = num_iterations
        self.step_size = step_size
        self.noise_std_dev = noise_std_dev

    def cost_function(self, joint_angles):
        """
        Ritorna un costo continuo che aumenta se il robot si avvicina agli ostacoli.
        Nessun early return, ma penalità crescenti.
        """
        x_list, y_list, z_list = zip(*self.robot.get_joint_positions(joint_angles))
        points = [np.array([x, y, z]) for x, y, z in zip(x_list, y_list, z_list)]
        interpolation_steps = 5
        sphere_radius = 0.15
        total_cost = 0.0

        for i in range(len(points) - 1):
            p_start = points[i]
            p_end = points[i + 1]
            for t in np.linspace(0, 1, interpolation_steps + 2):
                center = p_start + t * (p_end - p_start)
                for obstacle in self.obstacle_list:
                    shape = obstacle[0]
                    if shape == "sfera":
                        ox, oy, oz = obstacle[1]
                        obs_radius = obstacle[2]
                        dist = np.linalg.norm(center - np.array([ox, oy, oz]))
                        threshold = sphere_radius + obs_radius + 0.1  # piccolo margine di sicurezza
                        if dist < threshold:
                            total_cost += (threshold - dist) ** 2  # penalità quadratica

                    elif shape == "parallelepipedo":
                        cx, cy, cz = obstacle[1]
                        lx, ly, lz = obstacle[2]
                        min_box = np.array([cx - lx/2, cy - ly/2, cz - lz/2])
                        max_box = np.array([cx + lx/2, cy + ly/2, cz + lz/2])
                        closest_point = np.maximum(min_box, np.minimum(center, max_box))
                        dist = np.linalg.norm(center - closest_point)
                        threshold = sphere_radius + 0.1
                        if dist < threshold:
                            total_cost += (threshold - dist) ** 2  # penalità quadratica

        return total_cost



    def smoothness_cost(self, path):
        """Valuta la fluidità del percorso (somma delle accelerazioni quadrate)."""
        diff2 = np.diff(path, n=2, axis=0)  # differenza seconda
        return np.sum(np.square(diff2))


    

    def optimize(self):
        improved = False
        best_cost = self.evaluate_total_cost(self.path)

        for i in range(self.num_iterations):
            noise = np.random.normal(0, self.noise_std_dev, self.path.shape)
            new_path = self.path + noise
            new_path[0] = self.path[0]
            new_path[-1] = self.path[-1]

            total_cost = 0.0
            for q in new_path:
                total_cost += self.cost_function(q)
            total_cost += self.smoothness_cost(new_path)

            if total_cost < best_cost:
                self.path = new_path
                best_cost = total_cost
                improved = True

        # Verifica finale se il path ottimizzato è valido
        final_cost = self.evaluate_total_cost(self.path)
        if not improved or final_cost >= 1e5:
            rospy.loginfo("⚠️ STOMP fallito: nessun miglioramento o path troppo vicino agli ostacoli.")
            return None

        return self.path.tolist()

    def evaluate_total_cost(self, path):
        cost = sum([self.cost_function(q) for q in path])
        cost += self.smoothness_cost(path)
        return cost
