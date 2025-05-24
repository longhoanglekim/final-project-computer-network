from router import Router
from packet import Packet
import json

INFINITY = 16

class DVrouter(Router):
    def __init__(self, addr, heartbeat_time):
        super().__init__(addr)
        self.heartbeat_time = heartbeat_time
        self.last_time = 0

        self.distance_vector = {addr: 0}
        self.forwarding_table = {}
        self.neighbor_vectors = {}
        self.port_map = {}         
        self.addr_to_port = {}     

    def handle_packet(self, port, packet):
        if packet.is_traceroute:
            dest = packet.dst_addr
            if dest in self.forwarding_table:
                out_port = self.forwarding_table[dest][1]
                self.send(out_port, packet)
        else:
            neighbor = packet.src_addr
            vector = json.loads(packet.content)
            self.neighbor_vectors[neighbor] = vector

            updated = self.update_distance_vector()
            if updated:
                self.broadcast_distance_vector()

    def handle_new_link(self, port, endpoint, cost):
        self.port_map[port] = endpoint
        self.addr_to_port[endpoint] = port

        self.distance_vector[endpoint] = cost
        self.forwarding_table[endpoint] = (cost, port)

        if endpoint not in self.neighbor_vectors:
            self.neighbor_vectors[endpoint] = {}

        updated = self.update_distance_vector()
        if updated:
            self.broadcast_distance_vector()

    def handle_remove_link(self, port):
        if port in self.port_map:
            neighbor = self.port_map[port]

            if neighbor in self.neighbor_vectors:
                del self.neighbor_vectors[neighbor]

            self.distance_vector[neighbor] = INFINITY

            if neighbor in self.forwarding_table:
                del self.forwarding_table[neighbor]

            del self.port_map[port]
            del self.addr_to_port[neighbor]

            updated = self.update_distance_vector()
            if updated:
                self.broadcast_distance_vector()

    def handle_time(self, time_ms):
        if time_ms - self.last_time >= self.heartbeat_time:
            self.last_time = time_ms
            self.broadcast_distance_vector()

    def broadcast_distance_vector(self):
        content = json.dumps(self.distance_vector)
        for port, neighbor in self.port_map.items():
            packet = Packet(
                src_addr=self.addr,
                dst_addr=neighbor,
                content=content,
                kind='routing' 
            )
            self.send(port, packet)

    def update_distance_vector(self):
        updated = False
        new_vector = {self.addr: 0}  

        all_dests = set()
        for vec in self.neighbor_vectors.values():
            all_dests.update(vec.keys())
        all_dests.update(self.port_map.values())

        for dest in all_dests:
            if dest == self.addr:
                continue

            min_cost = INFINITY
            min_port = None

            for neighbor, vec in self.neighbor_vectors.items():
                port_to_neighbor = self.addr_to_port.get(neighbor)
                cost_to_neighbor = self.distance_vector.get(neighbor, INFINITY)
                neighbor_cost = vec.get(dest, INFINITY)
                total_cost = min(INFINITY, cost_to_neighbor + neighbor_cost)

                if total_cost < min_cost:
                    min_cost = total_cost
                    min_port = port_to_neighbor

            if dest in self.addr_to_port:
                direct_cost = self.distance_vector.get(dest, INFINITY)
                if direct_cost < min_cost:
                    min_cost = direct_cost
                    min_port = self.addr_to_port[dest]

            old_cost = self.distance_vector.get(dest, INFINITY)
            if old_cost != min_cost:
                updated = True

            new_vector[dest] = min_cost

            if min_cost < INFINITY and min_port is not None:
                self.forwarding_table[dest] = (min_cost, min_port)
            elif dest in self.forwarding_table:
                del self.forwarding_table[dest]

        self.distance_vector = new_vector
        return updated

    def __repr__(self):
        return f"DVrouter(addr={self.addr})"
