import numpy as np

class ChompOptimizer:
    def __init__(self, robot, initial_path, obstacle_list, alpha=0.01, lambda_obs=100.0, eta=0.5):
        self.robot = robot
        self.path = np.array(initial_path)
        self.obstacle_list = obstacle_list
        self.alpha = alpha
        self.lambda_obs = lambda_obs
        self.eta = eta

        if np.any(np.isnan(self.path)):
            raise ValueError("⚠️ Il path iniziale contiene NaN!")

    def compute_smoothness_gradient(self):
        grad = np.zeros_like(self.path)
        for i in range(1, len(self.path) - 1):
            grad[i] = 2 * self.path[i] - self.path[i - 1] - self.path[i + 1]
        return grad

    def compute_obstacle_gradient(self):
        grad = np.zeros_like(self.path)
        for i in range(1, len(self.path) - 1):
            q = self.path[i]

            if np.any(np.isnan(q)):
                print(f"⚠️ q contiene NaN: {q}")
                continue

            joint_positions = self.robot.get_joint_positions(q)
            if joint_positions is None or any(np.any(np.isnan(p)) for p in joint_positions):
                print(f"⚠️ Posizione non valida per q = {q}")
                continue

            for link_idx, pos in enumerate(joint_positions):
                pos = np.array(pos)
                grad_pot = np.zeros(3)

                for obs in self.obstacle_list:
                    if obs[0] == "sfera":
                        ox, oy, oz = obs[1]
                        r_obs = obs[2]
                        obs_pos = np.array([ox, oy, oz])
                        d = max(np.linalg.norm(pos - obs_pos), 1e-6)
                        if d < self.eta:
                            grad_U = (1 / d - 1 / self.eta) * (1 / d**3) * (pos - obs_pos)
                            grad_pot += grad_U

                    elif obs[0] == "parallelepipedo":
                        cx, cy, cz = obs[1]
                        lx, ly, lz = obs[2]
                        min_box = np.array([cx - lx/2, cy - ly/2, cz - lz/2])
                        max_box = np.array([cx + lx/2, cy + ly/2, cz + lz/2])
                        closest = np.maximum(min_box, np.minimum(pos, max_box))
                        d = max(np.linalg.norm(pos - closest), 1e-6)
                        if d < self.eta:
                            grad_U = (1 / d - 1 / self.eta) * (1 / d**3) * (pos - closest)
                            grad_pot += grad_U

                J = self.robot.get_jacobian(q, link_idx)
                if J is None or J.ndim != 2:
                    print(f"⚠️ Jacobiano non valido per link {link_idx} - q = {q}")
                    continue

                if J.shape[1] != self.path.shape[1]:
                    J_full = np.zeros((3, self.path.shape[1]))
                    J_full[:, :J.shape[1]] = J
                    J = J_full

                grad_q = J.T @ grad_pot
                if grad_q.shape != grad[i].shape:
                    print(f"⚠️ grad_q ha forma errata: {grad_q.shape}, atteso {grad[i].shape}")
                    continue

                grad[i] += grad_q

        return grad

    def optimize(self, num_iterations=50):
        for it in range(num_iterations):
            if np.any(np.isnan(self.path)):
                print(f"❌ Iterazione {it}: path contiene NaN! Interrompo.")
                return None

            grad_smooth = self.compute_smoothness_gradient()
            grad_obs = self.compute_obstacle_gradient()
            total_grad = grad_smooth + self.lambda_obs * grad_obs

            if np.any(np.isnan(total_grad)):
                print(f"❌ Iterazione {it}: total_grad contiene NaN!")
                return None

            self.path -= self.alpha * total_grad

        if np.any(np.isnan(self.path)):
            print("❌ Ottimizzazione terminata ma il path finale contiene NaN!")
            return None

        return self.path.tolist()
