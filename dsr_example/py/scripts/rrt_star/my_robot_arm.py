import PyKDL  # for kinematic and dynamic calculations of robots
import numpy as np
import rospy
import math
from PyKDL import ChainJntToJacSolver, JntArray, Chain, Jacobian
#class my robot arm
class My_RobotArm:




    def __init__(self, chain): # chain is the kinematic = how the robot's parts are connected).
        """
        Inizializza il braccio robotico dato un oggetto KDL Chain.
        :param chain: Catena cinematica del robot
        """
        self.chain = chain
        self.n_joints = chain.getNrOfJoints()  # Numero di giunti  ,  counts the number of joints and save this number 
        self.fk_solver = PyKDL.ChainFkSolverPos_recursive(chain)  # calcola la posizione finale (Forward Kinematics)
        self.ik_solver = PyKDL.ChainIkSolverPos_LMA(chain)  # calcola gli angoli dei giunti necessari per raggiungere una certa posizione (Inverse Kinematics)
        self.joint_limits = [
            (-2 * math.pi, 2 * math.pi),   # J1: ±360°
            (-2 * math.pi, 2 * math.pi),   # J2: ±360°
            (-math.radians(150), math.radians(150)),  # J3: ±150°
            (-2 * math.pi, 2 * math.pi),   # J4: ±360°
            (-2 * math.pi, 2 * math.pi),   # J5: ±360°
            (-2 * math.pi, 2 * math.pi)    # J6: ±360°
        ]







    def forward_kinematics(self, joint_angles): #starts with an input:A list of angles for each joint
        """
        Calcola la cinematica diretta (posizione dell'effettore finale) dato un set di angoli dei giunti.
        :param joint_angles: Lista o array di angoli dei giunti in radianti
        :return: Lista di coordinate [x, y, z] della posizione dell'effettore finale
        """
        # Imposta gli angoli dei giunti , creates arry to store the joint angles
        jnt_array = PyKDL.JntArray(self.n_joints)
        for i in range(self.n_joints):
            jnt_array[i] = joint_angles[i]
        
        # Calcola la cinematica diretta , calculates the robot ee position  based on joint angles
        ee_frame = PyKDL.Frame()
        if self.fk_solver.JntToCart(jnt_array, ee_frame) < 0:
            rospy.logerr("Errore nella cinematica diretta")
            return None
        
        # Estrai la posizione dell'effettore finale , xtracts the (x, y, z) position of the robot’s hand = Returns the final position
        position = ee_frame.p
        return [position.x(), position.y(), position.z()]
    

    
    def check_joint_limits(self, joint_angles):
        rospy.loginfo("Controllo dei limiti articolari")
        for i, angle in enumerate(joint_angles):
            joint_min, joint_max = self.joint_limits[i]
            if not (joint_min <= angle <= joint_max):
                return False
        return True

    # two inputs:target_pos: The desired position , target_rot (optional): The desired orientation
    def inverse_kinematics(self, target_pos, target_rot=None):
        """
        Calcola la cinematica inversa per ottenere gli angoli dei giunti dati una posizione (x, y, z) e opzionalmente un orientamento.
        :param target_pos: Posizione desiderata (x, y, z)
        :param target_rot: (opzionale) Matrice di rotazione desiderata
        :return: Lista degli angoli dei giunti (o None se non trovato)
        """
        # Definisci la configurazione iniziale dei giunti , creates a list of initial joint angles  
        q_init = PyKDL.JntArray(self.n_joints)
        for i in range(self.n_joints):
            q_init[i] = 0.0  # Configurazione di riposo , rest position and sets to 0 radians 

        # Definisci la posizione finale desiderata , creates a frame for the end-effector , sets its position using target_pos
        end_effector_frame = PyKDL.Frame()
        end_effector_frame.p = PyKDL.Vector(*target_pos)
        
        # Se fornito, aggiungi l'orientamento , orientation of end effector 
        if target_rot:
            if isinstance(target_rot, PyKDL.Rotation): # represent the orientation , it used directly = PyKDL
                end_effector_frame.M = target_rot  #  M = matrix for rotation ee 
            else:
                # Quando target_rot è una lista di rotazione [roll, pitch, yaw] , the code converts it into a rotation matrix first then applied it 
                roll, pitch, yaw = target_rot
                rotation = PyKDL.Rotation.RPY(roll, pitch, yaw)
                end_effector_frame.M = rotation

        # Risoluzione cinematica inversa, create an arry of joint angles 
        joint_result = PyKDL.JntArray(self.n_joints)
        if self.ik_solver.CartToJnt(q_init, end_effector_frame, joint_result) >= 0: # solver find the solution 
            return [joint_result[i] for i in range(self.n_joints)]
        else:        
            return None  # Nessuna soluzione trovata , solver can not find solution 












    def get_joint_positions(self, joint_angles, resolution=3):
        """
        Calcola la posizione di tutti i giunti e dei punti intermedi per rappresentare meglio la struttura del braccio.

        :param joint_angles: Lista di angoli dei giunti
        :param resolution: Numero di punti intermedi tra un giunto e l'altro
        :return: Lista delle coordinate [x, y, z] di giunti e punti intermedi
        """
        jnt_array = PyKDL.JntArray(self.n_joints)   # space to store the angles of the joints, starts with a starting position of the robot , use forward kinematics 
        for i in range(self.n_joints):             
            jnt_array[i] = joint_angles[i]

        trans = PyKDL.Frame.Identity()   #create an identity frame to calculate the position of each joint 

        # Lista delle posizioni reali dei giunti ,  list of real position of each joint 
        real_joint_positions = []

        for i in range(self.chain.getNrOfSegments()):
            segment = self.chain.getSegment(i)
            joint = segment.getJoint()

            if joint.getType() != getattr(PyKDL.Joint, "None"):
                self.fk_solver.JntToCart(jnt_array, trans, i + 1)
                real_joint_positions.append([trans.p.x(), trans.p.y(), trans.p.z()])

        return real_joint_positions








    
    def get_joint_angles_from_pose(self, pose,orientation):
        """
        Calcola gli angoli dei giunti necessari per raggiungere una posa cartesiana specificata.

        :param pose: Posa cartesiana dell'end-effector [x, y, z, qx, qy, qz, qw]
        :return: Lista degli angoli dei giunti
        """
        target_frame = PyKDL.Frame(
            PyKDL.Rotation.Quaternion(orientation[1], orientation[2], orientation[3], orientation[4]),
            PyKDL.Vector(pose[0], pose[1], pose[2])
        )

        jnt_array = PyKDL.JntArray(self.n_joints)
        initial_guess = PyKDL.JntArray(self.n_joints)  # Può essere tutto 0 o l'ultima configurazione nota
        result = self.ik_solver.CartToJnt(initial_guess, target_frame, jnt_array)

        if result < 0:
            raise ValueError("Cinematica inversa fallita per la posa specificata.")

        return [jnt_array[i] for i in range(self.n_joints)]

        
        





      # Nuova funzione per punti interpolati
    def get_interpolated_link_points(self, joint_angles, resolution=5):
        """
        Calcola la posizione dei giunti e dei punti intermedi lungo i link.

        :param joint_angles: Lista di angoli dei giunti
        :param resolution: Numero totale di punti per link (inclusi inizio e fine). Minimo 2.
        :return: Lista delle coordinate [x, y, z] di giunti e punti intermedi
        """
        if resolution < 2:
            resolution = 2 # Necessita almeno di inizio e fine

        jnt_array = PyKDL.JntArray(self.n_joints)
        for i in range(self.n_joints):
            jnt_array[i] = joint_angles[i]

        all_points = []
        frame_start = PyKDL.Frame.Identity() # Inizia dalla base

        # Aggiungi la posizione base come primo punto
        all_points.append([frame_start.p.x(), frame_start.p.y(), frame_start.p.z()])

        for i in range(self.chain.getNrOfSegments()):
            frame_end = PyKDL.Frame() # Frame alla fine del segmento i
            ret = self.fk_solver.JntToCart(jnt_array, frame_end, i + 1) # Calcola posa alla fine del segmento i

            if ret < 0:
                rospy.logerr(f"Errore FK nel calcolare la posa per il segmento {i+1}")
                continue # Salta questo segmento se c'è errore

            # Punti iniziale e finale del link corrente (segmento i)
            p_start = np.array([frame_start.p.x(), frame_start.p.y(), frame_start.p.z()])
            p_end = np.array([frame_end.p.x(), frame_end.p.y(), frame_end.p.z()])

            # Aggiungi punti interpolati (escludendo p_start già aggiunto dal ciclo precedente)
            # e includendo p_end. Vogliamo 'resolution' punti totali PER link.
            for j in range(1, resolution): # Da 1 a resolution-1 per i punti intermedi
                t = float(j) / (resolution -1) # Parametro di interpolazione da >0 a 1.0
                p_inter = p_start + t * (p_end - p_start)
                all_points.append(list(p_inter))

            # Aggiorna frame_start per il prossimo ciclo
            frame_start = PyKDL.Frame(frame_end) # Usa la fine corrente come inizio del prossimo

        # Potrebbe esserci un punto duplicato alla fine se l'ultimo segmento è incluso.
        # Possiamo rimuovere duplicati se necessario, ma per la visualizzazione
        # con marker sovrapposti potrebbe non essere un grosso problema.


    def get_jacobian(self, joint_angles, link_index):
        """
        Calcola il Jacobiano per il link indicato, usando una sottocatena manuale.
        """
        if isinstance(joint_angles, list):
            joint_angles = np.array(joint_angles)

        q_kdl = JntArray(len(joint_angles))
        for i in range(len(joint_angles)):
            q_kdl[i] = joint_angles[i]

        # Costruisci manualmente la sottocatena
        sub_chain = Chain()
        for i in range(link_index + 1):
            sub_chain.addSegment(self.chain.getSegment(i))

        # Crea solver per la sottocatena
        solver = ChainJntToJacSolver(sub_chain)

        # Inizializza oggetto Jacobian
        jacobian_kdl = Jacobian(sub_chain.getNrOfJoints())

        # Calcolo corretto del Jacobiano
        solver.JntToJac(q_kdl, jacobian_kdl)

        # Conversione in numpy array
        rows = jacobian_kdl.rows()
        cols = jacobian_kdl.columns()
        jacobian = np.zeros((rows, cols))
        for i in range(rows):
            for j in range(cols):
                jacobian[i, j] = jacobian_kdl[i, j]

        return jacobian[0:3, :]  # Solo componente lineare (x, y, z)