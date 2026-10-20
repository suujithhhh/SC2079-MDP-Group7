import math
import time
import tkinter as tk
from typing import Iterable, Optional, Tuple

from config import (
    ANIMATION_STEP_CM,
    ARENA_CM,
    GRID_CELL_CM,
    MARGIN_PX,
    OBSTACLE_CM,
    PANEL_WIDTH_PX,
    SCALE,
    TURN_RADIUS_CM,
)
from b3_optimizer import B3RouteResult, find_b2_route, optimise_b3_route
from config import DEFAULT_PLANNER_CONFIG, PlannerConfig
from footprint import camera_position, footprint_corners, local_to_world, pose_is_safe
from models import Obstacle, Pose
from motion import move_curve, move_straight
from map_editor import MapEditorModel
from planner_interface import CarPathPlanner, plan_car_path
from route_engine import RouteEngine
from route_validation import route_is_continuous
from targets import IMAGE_FACE_NAMES, face_is_available, obstacle_face_edge, target_for_face, viewing_target

SIDEBAR_BG = "#f8fafc"
TEXT_FG = "#111827"
HEADING_FG = "#1f2937"
SUCCESS_FG = "#166534"
ERROR_FG = "#b91c1c"
BUTTON_BG = "#ffffff"
BUTTON_DISABLED_BG = "#d1d5db"
BUTTON_DISABLED_FG = "#6b7280"


def calculate_arena_view(width: int, height: int, padding: int = 35) -> Tuple[float, float, float]:
    side = max(1, min(width, height) - padding * 2)
    return side / ARENA_CM, (width - side) / 2, (height - side) / 2


class MDPSimulator:
    def __init__(
        self,
        start_pose: Pose,
        obstacles: Iterable[Obstacle],
        car_planner: CarPathPlanner = plan_car_path,
        planner_config: PlannerConfig = DEFAULT_PLANNER_CONFIG,
    ) -> None:
        self.start_pose = start_pose
        self.pose = Pose(start_pose.x, start_pose.y, start_pose.theta)
        self.obstacles = list(obstacles)
        self.car_planner = car_planner
        self.planner_config = planner_config
        self.demo_running = False
        self.demo_after_id: Optional[int] = None
        self.demo_index = 0
        self.demo_remaining = 0.0
        self.demo_commands = [
            ("F", 45.0),
            ("L", math.radians(45)),
            ("F", 50.0),
            ("R", math.radians(90)),
            ("F", 35.0),
            ("B", 18.0),
        ]

        self.root = tk.Tk()
        self.root.title("MDP Path Planner — Assessor Console")
        self.root.configure(background="#f8fafc")
        self.root.minsize(1200, 800)
        self.root.geometry("1280x820")
        self.root.rowconfigure(0, weight=1)
        self.root.columnconfigure(0, weight=1)
        arena_px = ARENA_CM * SCALE
        self.main_frame = tk.Frame(self.root, background="#f8fafc")
        self.main_frame.grid(row=0, column=0, sticky="nsew")
        self.main_frame.rowconfigure(0, weight=1)
        self.main_frame.columnconfigure(0, weight=1)
        self.main_frame.columnconfigure(1, weight=0, minsize=420)
        self.canvas = tk.Canvas(
            self.main_frame,
            width=arena_px + MARGIN_PX * 2,
            height=arena_px + MARGIN_PX * 2,
            background="#f8fafc",
            highlightthickness=0,
        )
        self.canvas.grid(row=0, column=0, sticky="nsew")
        self.sidebar_host = tk.Frame(
            self.main_frame, background=SIDEBAR_BG, width=PANEL_WIDTH_PX)
        self.sidebar_host.grid(row=0, column=1, sticky="nsew")
        self.sidebar_host.rowconfigure(0, weight=1)
        self.sidebar_host.columnconfigure(0, weight=1)
        self.sidebar_canvas = tk.Canvas(
            self.sidebar_host, background=SIDEBAR_BG, highlightthickness=0, width=PANEL_WIDTH_PX
        )
        self.sidebar_scrollbar = tk.Scrollbar(
            self.sidebar_host, orient="vertical", command=self.sidebar_canvas.yview
        )
        self.sidebar_canvas.configure(
            yscrollcommand=self.sidebar_scrollbar.set)
        self.sidebar_canvas.grid(row=0, column=0, sticky="nsew")
        self.sidebar_scrollbar.grid(row=0, column=1, sticky="ns")
        self.side_panel = tk.Frame(
            self.sidebar_canvas, background=SIDEBAR_BG, padx=16, pady=14)
        self.sidebar_window = self.sidebar_canvas.create_window(
            (0, 0), window=self.side_panel, anchor="nw"
        )
        self.side_panel.bind("<Configure>", self.on_sidebar_content_resize)
        self.sidebar_canvas.bind("<Configure>", self.on_sidebar_canvas_resize)
        self.canvas_scale = SCALE
        self.arena_origin_x = MARGIN_PX
        self.arena_origin_y = MARGIN_PX
        self.canvas.bind("<Configure>", self.on_canvas_resize)
        self.root.bind("<space>", self.start_assessment)
        self.root.bind("d", self.toggle_demo)
        self.root.bind("D", self.toggle_demo)
        self.root.bind("<Up>", lambda _event: self.manual_straight(5))
        self.root.bind("<Down>", lambda _event: self.manual_straight(-5))
        self.root.bind(
            "<Left>", lambda _event: self.manual_curve(math.radians(15)))
        self.root.bind(
            "<Right>", lambda _event: self.manual_curve(-math.radians(15)))
        self.root.bind("<Escape>", lambda _event: self.root.destroy())
        self.root.bind("<Delete>", self.delete_selected_obstacle)
        self.root.bind("<BackSpace>", self.delete_selected_obstacle)
        self.canvas.bind("<ButtonPress-1>", self.begin_obstacle_drag)
        self.canvas.bind("<ButtonRelease-1>", self.end_obstacle_drag)
        self.canvas.bind("<Button-3>", self.cycle_obstacle_face)

        self.editor = MapEditorModel(self.obstacles)
        self.dragged_obstacle_id = None
        self.drag_start_world = None
        self.selected_obstacle_id = None
        self.edit_error = ""
        self.operator_status = self.placement_status()
        self.plan_succeeded = False
        self.physical_path = []
        self.physical_path_index = 0
        self.physical_target_indices = []
        self.physical_after_id = None
        self.assessment_active = False
        self.active_mode = None
        self.animation_started_at = None
        self.animation_elapsed_s = 0.0
        self.route_paused = False
        self.b2_result = B3RouteResult()
        self.b2_comparison_time_s: Optional[float] = None

        self.build_sidebar()

        self.assessment_ready = self.map_is_assessment_ready()
        self.route_engine = RouteEngine(self.start_pose, [])
        self.route_order = self.route_engine.route_order
        self.route_targets = self.route_engine.route_targets
        self.route_distance = self.route_engine.total_distance
        self.route_order_map = {
            obstacle_id: index + 1 for index, obstacle_id in enumerate(self.route_order)}
        self.route_index = self.route_engine.route_index
        self.route_running = self.route_engine.running
        self.route_after_id = None
        self.visited_targets = set(self.route_engine.visited_ids)
        self.completed_targets = self.route_engine.completed_targets
        self.b3_result = B3RouteResult()

        self.redraw()

    def build_sidebar(self) -> None:
        def button(parent: tk.Misc, **options: object) -> tk.Button:
            return tk.Button(
                parent,
                background=BUTTON_BG,
                foreground=TEXT_FG,
                activebackground=BUTTON_BG,
                activeforeground=TEXT_FG,
                disabledforeground=BUTTON_DISABLED_FG,
                **options,
            )

        def section(title: str) -> tk.LabelFrame:
            frame = tk.LabelFrame(
                self.side_panel,
                text=title,
                background=SIDEBAR_BG,
                foreground=HEADING_FG,
                font=("Arial", 11, "bold"),
                padx=12,
                pady=8,
            )
            frame.pack(fill="x", pady=(0, 10))
            return frame

        status = section("Status")
        self.status_var, self.error_var = tk.StringVar(), tk.StringVar()
        self.status_label = tk.Label(
            status, textvariable=self.status_var, anchor="w", justify="left", wraplength=470,
            background=SIDEBAR_BG, foreground=TEXT_FG, font=(
                "Arial", 15, "bold")
        )
        self.status_label.pack(fill="x")
        tk.Label(status, textvariable=self.error_var, anchor="w", justify="left", wraplength=470,
                 background=SIDEBAR_BG, foreground=ERROR_FG,
                 font=("Arial", 10, "bold")).pack(fill="x")

        controls = section("B2 / B3 controls")
        self.start_button = button(
            controls, text="RUN B2", command=self.run_b2, font=("Arial", 13, "bold"), height=2,
        )
        self.start_button.pack(side="left", fill="x", expand=True, padx=(0, 8))
        self.b3_button = button(
            controls, text="RUN B3\nSHORTEST", command=self.run_b3,
            font=("Arial", 11, "bold"), height=2,
        )
        self.b3_button.pack(side="left", fill="x", expand=True, padx=(0, 8))
        self.reset_map_button = button(
            controls, text="RESET MAP", command=self.reset_map, font=("Arial", 10, "bold")
        )
        self.reset_map_button.pack(side="left", fill="x", expand=True)

        selected = section("Selected obstacle")
        self.selected_var, self.face_error_var = tk.StringVar(), tk.StringVar()
        tk.Label(selected, textvariable=self.selected_var, anchor="w", background=SIDEBAR_BG,
                 foreground=TEXT_FG,
                 font=("Arial", 11, "bold")).pack(fill="x", pady=(0, 6))
        face_row = tk.Frame(selected, background=SIDEBAR_BG)
        face_row.pack(fill="x")
        self.face_buttons = {}
        for face, name in IMAGE_FACE_NAMES.items():
            face_button = button(
                face_row, text=name, command=lambda value=face: self.select_obstacle_face(
                    value),
                font=("Arial", 10, "bold"),
            )
            face_button.pack(side="left", fill="x", expand=True, padx=3)
            self.face_buttons[face] = face_button
        tk.Label(selected, textvariable=self.face_error_var, anchor="w", justify="left", wraplength=470,
                 background=SIDEBAR_BG, foreground=ERROR_FG,
                 font=("Arial", 9)).pack(fill="x", pady=(5, 0))

        obstacles = section("Obstacles")
        self.obstacle_vars = [tk.StringVar() for _ in range(5)]
        for variable in self.obstacle_vars:
            tk.Label(obstacles, textvariable=variable, anchor="w", background=SIDEBAR_BG,
                     foreground=TEXT_FG,
                     font=("Arial", 11)).pack(fill="x", pady=1)

        route = section("Planned route")
        self.route_summary_var = tk.StringVar()
        tk.Label(route, textvariable=self.route_summary_var, anchor="w", justify="left",
                 background=SIDEBAR_BG, foreground=TEXT_FG, font=("Arial", 11)).pack(fill="x")
        route_buttons = tk.Frame(route, background=SIDEBAR_BG)
        route_buttons.pack(fill="x", pady=(7, 0))
        self.details_button = button(
            route_buttons, text="Show route details", command=self.show_route_details, state=tk.DISABLED
        )
        self.details_button.pack(side="left")
        self.show_b2_var, self.show_b3_var = tk.BooleanVar(
            value=False), tk.BooleanVar(value=False)
        tk.Checkbutton(route_buttons, text="B2", variable=self.show_b2_var, command=self.redraw,
                       background=SIDEBAR_BG, foreground=TEXT_FG,
                       activebackground=SIDEBAR_BG, activeforeground=TEXT_FG,
                       disabledforeground=BUTTON_DISABLED_FG).pack(side="left", padx=(12, 0))
        tk.Checkbutton(route_buttons, text="Order", variable=self.show_b3_var, command=self.redraw,
                       background=SIDEBAR_BG, foreground=TEXT_FG,
                       activebackground=SIDEBAR_BG, activeforeground=TEXT_FG,
                       disabledforeground=BUTTON_DISABLED_FG).pack(side="left", padx=(8, 0))

        playback_buttons = tk.Frame(route, background=SIDEBAR_BG)
        playback_buttons.pack(fill="x", pady=(7, 0))
        self.previous_step_button = button(
            playback_buttons, text="◀ Previous", command=self.previous_route_step
        )
        self.previous_step_button.pack(
            side="left", fill="x", expand=True, padx=(0, 5))
        self.pause_button = button(
            playback_buttons, text="Pause", command=self.toggle_route_pause
        )
        self.pause_button.pack(side="left", fill="x", expand=True, padx=5)
        self.next_step_button = button(
            playback_buttons, text="Next ▶", command=self.next_route_step
        )
        self.next_step_button.pack(
            side="left", fill="x", expand=True, padx=(5, 0))

        help_section = section("Manual controls")
        self.manual_var = tk.StringVar()
        tk.Label(help_section, textvariable=self.manual_var, anchor="w", justify="left",
                 background=SIDEBAR_BG, foreground=TEXT_FG, font=("Arial", 9)).pack(fill="x")
        self.panel_sections = [status, controls,
                               selected, obstacles, route, help_section]

    def show_route_details(self) -> None:
        if not self.plan_succeeded:
            return
        window = tk.Toplevel(self.root)
        window.title("Route details")
        window.geometry("760x420")
        commands = [
            f"{command.kind} {command.value:.2f}" for leg in self.b3_result.leg_results for command in leg.commands]
        text = tk.Text(window, wrap="word", font=(
            "Menlo", 11), padx=12, pady=12)
        scrollbar = tk.Scrollbar(window, command=text.yview)
        text.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side="right", fill="y")
        text.pack(side="left", fill="both", expand=True)
        text.insert("1.0", "Full command sequence\n\n" + "  ".join(commands))
        text.configure(state=tk.DISABLED)

    def on_sidebar_content_resize(self, _event: object) -> None:
        self.sidebar_canvas.configure(
            scrollregion=self.sidebar_canvas.bbox("all"))

    def on_sidebar_canvas_resize(self, event: object) -> None:
        self.sidebar_canvas.itemconfigure(
            self.sidebar_window, width=event.width)

    def on_canvas_resize(self, event: object) -> None:
        self.canvas_scale, self.arena_origin_x, self.arena_origin_y = calculate_arena_view(
            event.width, event.height
        )
        self.redraw()

    def world_to_screen(self, x: float, y: float) -> Tuple[float, float]:
        sx = self.arena_origin_x + x * self.canvas_scale
        sy = self.arena_origin_y + (ARENA_CM - y) * self.canvas_scale
        return sx, sy

    def redraw(self) -> None:
        self.canvas.delete("all")
        self.draw_arena()
        self.draw_start_zone()
        self.draw_obstacles_and_targets()
        self.draw_robot()
        self.draw_info_panel()
        start_enabled = self.assessment_ready and not self.assessment_active
        for run_button in (self.start_button, self.b3_button):
            run_button.configure(
                state=tk.NORMAL if start_enabled else tk.DISABLED,
                background=BUTTON_BG if start_enabled else BUTTON_DISABLED_BG,
            )
        has_path = self.plan_succeeded and bool(self.physical_path)
        previous_enabled = has_path and self.physical_path_index > 0
        next_enabled = has_path and self.physical_path_index + \
            1 < len(self.physical_path)
        for control, enabled in (
            (self.previous_step_button, previous_enabled),
            (self.pause_button, next_enabled),
            (self.next_step_button, next_enabled),
        ):
            control.configure(
                state=tk.NORMAL if enabled else tk.DISABLED,
                background=BUTTON_BG if enabled else BUTTON_DISABLED_BG,
            )
        self.pause_button.configure(
            text="Resume" if self.route_paused else "Pause")

    def draw_arena(self) -> None:
        left, top = self.world_to_screen(0, ARENA_CM)
        right, bottom = self.world_to_screen(ARENA_CM, 0)
        self.canvas.create_rectangle(
            left, top, right, bottom, fill="#ffffff", outline="#0f172a", width=3)

        for coordinate in range(0, ARENA_CM + 1, GRID_CELL_CM):
            x1, y1 = self.world_to_screen(coordinate, 0)
            x2, y2 = self.world_to_screen(coordinate, ARENA_CM)
            self.canvas.create_line(x1, y1, x2, y2, fill="#e2e8f0")
            x3, y3 = self.world_to_screen(0, coordinate)
            x4, y4 = self.world_to_screen(ARENA_CM, coordinate)
            self.canvas.create_line(x3, y3, x4, y4, fill="#e2e8f0")

        self.canvas.create_text(
            left, bottom + 18, text="0 cm", fill="#475569", anchor="w")
        self.canvas.create_text(
            right, bottom + 18, text="200 cm", fill="#475569", anchor="e")

    def draw_start_zone(self) -> None:
        left, top = self.world_to_screen(0, 40)
        right, bottom = self.world_to_screen(40, 0)
        self.canvas.create_rectangle(
            left, top, right, bottom, fill="#dbeafe", outline="#2563eb", width=2)
        self.canvas.create_text(
            (left + right) / 2, (top + bottom) / 2, text="START\n40 x 40 cm", fill="#1d4ed8")

    def draw_obstacles_and_targets(self) -> None:
        # Draw the collision-safe A* path the robot actually follows.  Connecting
        # only the viewing targets produces misleading lines through obstacles.
        route_points = []
        for path in self.route_engine.route_paths if self.plan_succeeded and self.show_b2_var.get() else []:
            for waypoint in path:
                point = self.world_to_screen(waypoint.x, waypoint.y)
                if not route_points or point != route_points[-1]:
                    route_points.append(point)
        if len(route_points) >= 2:
            coordinates = [
                coordinate for point in route_points for coordinate in point]
            self.canvas.create_line(
                *coordinates, fill="#7c3aed", width=3, smooth=False)

        selected = [self.world_to_screen(self.start_pose.x, self.start_pose.y)]
        selected.extend(self.world_to_screen(target.x, target.y)
                        for target in self.b3_result.target_poses)
        if len(selected) >= 2 and self.show_b3_var.get():
            self.canvas.create_line(
                *[value for point in selected for value in point], fill="#16a34a", width=2, dash=(7, 5))
        executable = []
        if route_is_continuous(self.b3_result.leg_results, self.planner_config):
            for leg in self.b3_result.leg_results:
                executable.extend(self.world_to_screen(pose.x, pose.y)
                                  for pose in leg.poses)
        if len(executable) >= 2:
            self.canvas.create_line(
                *[value for point in executable for value in point],
                fill="#06b6d4",
                width=3,
                smooth=True,
                splinesteps=24,
            )

        for obstacle in self.obstacles:
            # The footprint is asymmetric about the rear reference point, so the
            # keep-out region for the pose depends on heading and cannot be drawn
            # as one rectangle. Show the safety-margin band the physical footprint
            # must not enter instead.
            clearance = self.planner_config.safety_margin_cm
            virtual_left, virtual_top = self.world_to_screen(
                obstacle.x - clearance, obstacle.y + OBSTACLE_CM + clearance)
            virtual_right, virtual_bottom = self.world_to_screen(
                obstacle.x + OBSTACLE_CM + clearance, obstacle.y - clearance)
            self.canvas.create_rectangle(
                virtual_left,
                virtual_top,
                virtual_right,
                virtual_bottom,
                fill="",
                outline="#ef4444",
                width=2,
                dash=(4, 3),
            )

            left, top = self.world_to_screen(
                obstacle.x, obstacle.y + OBSTACLE_CM)
            right, bottom = self.world_to_screen(
                obstacle.x + OBSTACLE_CM, obstacle.y)
            selected_outline = "#eab308" if obstacle.obstacle_id == self.selected_obstacle_id else "#0f172a"
            self.canvas.create_rectangle(left, top, right, bottom, fill="#475569", outline=selected_outline,
                                         width=4 if obstacle.obstacle_id == self.selected_obstacle_id else 2)
            self.canvas.create_text((left + right) / 2, (top + bottom) / 2, text=str(
                obstacle.obstacle_id), fill="white", font=("Arial", 12, "bold"))

            if obstacle.image_face:
                edge_start, edge_end = obstacle_face_edge(
                    obstacle, obstacle_cm=self.planner_config.obstacle_cm
                )
                line = (*self.world_to_screen(*edge_start),
                        *self.world_to_screen(*edge_end))
                self.canvas.create_line(*line, fill="#f97316", width=5)

                target = viewing_target(obstacle, self.planner_config)
                tx, ty = self.world_to_screen(target.x, target.y)
                self.canvas.create_oval(
                    tx - 6, ty - 6, tx + 6, ty + 6, fill="#16a34a", outline="white", width=2)
            else:
                target = None
            order = self.route_order_map.get(obstacle.obstacle_id)
            if order is not None:
                label = str(order)
                fill = "#1d4ed8" if obstacle.obstacle_id in self.visited_targets else "#15803d"
                self.canvas.create_text(
                    tx, ty - 15, text=label, fill=fill, font=("Arial", 9, "bold"))
                if obstacle.obstacle_id in self.visited_targets:
                    self.canvas.create_text(
                        tx, ty + 15, text="VISITED", fill="#7c3aed", font=("Arial", 8, "bold"))
            elif target is not None:
                self.canvas.create_text(
                    tx, ty - 15, text=f"T{obstacle.obstacle_id}", fill="#15803d", font=("Arial", 9, "bold"))

    def draw_robot(self) -> None:
        # Physical footprint (no margin) about the rear reference point.
        corners = []
        for x, y in footprint_corners(self.pose, self.planner_config, include_margin=False):
            corners.extend(self.world_to_screen(x, y))
        self.canvas.create_polygon(
            *corners,
            fill="#2563eb",
            outline="#1e3a8a",
            width=2,
        )

        rx, ry = self.world_to_screen(self.pose.x, self.pose.y)
        front = self.planner_config.robot_front_extent_cm
        tip_x, tip_y = self.world_to_screen(
            *local_to_world(self.pose, front * 0.9, 0.0))
        self.canvas.create_line(rx, ry, tip_x, tip_y,
                                fill="white", width=4, arrow=tk.LAST)
        # Rear reference point (planner pose) and camera lens.
        self.canvas.create_oval(rx - 4, ry - 4, rx + 4, ry + 4,
                                fill="#f97316", outline="white", width=1)
        lens_x, lens_y = self.world_to_screen(
            *camera_position(self.pose, self.planner_config))
        self.canvas.create_oval(lens_x - 3, lens_y - 3, lens_x + 3, lens_y + 3,
                                fill="#facc15", outline="#1e3a8a", width=1)
        label_y = max(y for y in corners[1::2]) + 12
        self.canvas.create_text(
            sum(corners[0::2]) / 4, label_y, text="Robot", fill="#1e3a8a", font=("Arial", 9, "bold"))

    def draw_info_panel(self) -> None:
        heading_deg = (math.degrees(self.pose.theta) + 360) % 360
        self.status_var.set(self.operator_status)
        self.status_label.configure(
            foreground=SUCCESS_FG if self.assessment_ready or self.plan_succeeded else TEXT_FG
        )
        self.error_var.set(self.edit_error)
        selected = self.selected_obstacle()
        self.selected_var.set(
            f"Selected obstacle: {selected.obstacle_id}" if selected else "None selected")
        unavailable = []
        for face, button in self.face_buttons.items():
            available = selected is not None and face_is_available(
                selected, face, self.planner_config)
            chosen = selected is not None and selected.image_face == face
            button.configure(
                state=tk.NORMAL if available else tk.DISABLED,
                relief=tk.SUNKEN if chosen else tk.RAISED,
                background=BUTTON_BG if available else BUTTON_DISABLED_BG,
            )
            if selected is not None and not available:
                unavailable.append(IMAGE_FACE_NAMES[face])
        self.face_error_var.set(
            "; ".join(f"{name} unavailable: target outside arena" for name in unavailable))

        obstacles = sorted(self.obstacles, key=lambda item: item.obstacle_id)
        for index, variable in enumerate(self.obstacle_vars):
            if index < len(obstacles):
                obstacle = obstacles[index]
                face = IMAGE_FACE_NAMES.get(obstacle.image_face, "Unset")
                variable.set(
                    f"{obstacle.obstacle_id}. ({obstacle.x:g}, {obstacle.y:g}) | {face}")
            elif index == 0:
                variable.set("No obstacles placed")
            else:
                variable.set("")

        if self.plan_succeeded:
            segment_count = sum(len(leg.commands)
                                for leg in self.b3_result.leg_results)
            elapsed = self.animation_elapsed_s
            route_finished = (
                bool(self.route_targets)
                and self.completed_targets == len(self.route_targets)
            )
            remaining = (
                0.0
                if route_finished
                else max(0.0, self.b3_result.estimated_time_s - elapsed)
            )
            if self.active_mode == "B3":
                comparison = self.b2_comparison_time_s
                coverage = (
                    "Complete Hamiltonian route"
                    if self.b3_result.is_complete
                    else f"Maximum coverage: {len(self.route_order)}/5; "
                    f"skipped {self.b3_result.skipped_obstacle_ids}"
                )
                summary = (
                    "Active mode: B3 shortest-time route\n"
                    f"{coverage}\n"
                    f"Orders evaluated: {self.b3_result.orders_considered}\n"
                    f"Valid orders: {self.b3_result.valid_orders}\n"
                    f"Rejected orders: {self.b3_result.rejected_orders}\n"
                    f"Selected shortest order: {' → '.join(map(str, self.route_order))}\n"
                    f"Estimated total time: {self.b3_result.estimated_time_s:.1f} s\n"
                    + (
                        f"B2 comparison time: {comparison:.1f} s\n"
                        f"B3 improvement (B3 - B2): "
                        f"{elapsed - comparison:.1f} s\n"
                        if comparison is not None
                        else "B2 comparison time: Run B2 first\n"
                             "B3 improvement (B3 - B2): unavailable\n"
                    )
                )
            else:
                coverage = (
                    "Complete route"
                    if self.b3_result.is_complete
                    else f"Maximum coverage: {len(self.route_order)}/5; "
                    f"skipped {self.b3_result.skipped_obstacle_ids}"
                )
                summary = (
                    "Active mode: B2 route\n"
                    f"{coverage}\n"
                    f"Visit order: {' → '.join(map(str, self.route_order))}\n"
                    f"Estimated total time: {self.b3_result.estimated_time_s:.1f} s\n"
                )
            self.route_summary_var.set(
                summary
                + f"Completed: {self.completed_targets}/5\n"
                + f"Elapsed time: {elapsed:.1f} s\n"
                + f"Remaining estimate: {remaining:.1f} s\n"
                + f"Commands: {segment_count} segments"
            )
            self.details_button.configure(
                state=tk.NORMAL, background=BUTTON_BG)
        else:
            self.route_summary_var.set("No route planned")
            self.details_button.configure(
                state=tk.DISABLED, background=BUTTON_DISABLED_BG)
        self.manual_var.set(
            f"Robot rear ref ({self.pose.x:.1f}, {self.pose.y:.1f}) cm, {heading_deg:.0f}°\n"
            "Arrow keys: forward / reverse / curve    D: demo    Esc: close"
        )

    def screen_to_world(self, screen_x: float, screen_y: float) -> Tuple[float, float]:
        return (
            (screen_x - self.arena_origin_x) / self.canvas_scale,
            ARENA_CM - (screen_y - self.arena_origin_y) / self.canvas_scale,
        )

    def begin_obstacle_drag(self, event: object) -> None:
        x, y = self.screen_to_world(event.x, event.y)
        obstacle = self.editor.obstacle_at(x, y)
        if obstacle is None and len(self.obstacles) < 5:
            cell_x = math.floor(x / GRID_CELL_CM) * GRID_CELL_CM
            cell_y = math.floor(y / GRID_CELL_CM) * GRID_CELL_CM
            added = (
                self.editor.add(cell_x, cell_y)
                if 0 <= x < ARENA_CM and 0 <= y < ARENA_CM
                else None
            )
            if added is None:
                self.edit_error = "Choose an empty position outside the start zone."
            else:
                self.selected_obstacle_id = added.obstacle_id
                self.recompute_routes_after_edit()
            self.redraw()
            return
        self.dragged_obstacle_id = obstacle.obstacle_id if obstacle else None
        self.drag_start_world = (x, y) if obstacle else None
        self.selected_obstacle_id = self.dragged_obstacle_id
        self.redraw()

    def end_obstacle_drag(self, event: object) -> None:
        if self.dragged_obstacle_id is None:
            return
        x, y = self.screen_to_world(event.x, event.y)
        if self.drag_start_world and math.hypot(x - self.drag_start_world[0], y - self.drag_start_world[1]) < 2:
            self.dragged_obstacle_id = None
            self.drag_start_world = None
            self.redraw()
            return
        if self.editor.move(self.dragged_obstacle_id, x - OBSTACLE_CM / 2, y - OBSTACLE_CM / 2):
            self.recompute_routes_after_edit()
        else:
            self.edit_error = "Invalid placement: arena, start zone, or obstacle overlap"
        self.dragged_obstacle_id = None
        self.drag_start_world = None
        self.redraw()

    def cycle_obstacle_face(self, event: object) -> None:
        x, y = self.screen_to_world(event.x, event.y)
        obstacle = self.editor.obstacle_at(x, y)
        if obstacle:
            self.selected_obstacle_id = obstacle.obstacle_id
            current = obstacle.image_face if obstacle.image_face in IMAGE_FACE_NAMES else None
            faces = list(IMAGE_FACE_NAMES)
            start = faces.index(current) + 1 if current else 0
            for offset in range(len(faces)):
                face = faces[(start + offset) % len(faces)]
                if face_is_available(obstacle, face, self.planner_config):
                    self.editor.set_face(obstacle.obstacle_id, face)
                    self.recompute_routes_after_edit()
                    break
            self.redraw()

    def selected_obstacle(self) -> Optional[Obstacle]:
        return next(
            (item for item in self.obstacles if item.obstacle_id ==
             self.selected_obstacle_id),
            None,
        )

    def map_is_assessment_ready(self) -> bool:
        return len(self.obstacles) == 5 and all(
            obstacle.image_face in IMAGE_FACE_NAMES
            and face_is_available(obstacle, obstacle.image_face, self.planner_config)
            for obstacle in self.obstacles
        )

    def select_obstacle_face(self, image_face: str) -> None:
        obstacle = self.selected_obstacle()
        if obstacle is None:
            return
        name = IMAGE_FACE_NAMES[image_face]
        if not face_is_available(obstacle, image_face, self.planner_config):
            self.edit_error = f"{name} is unavailable: target would be outside the arena."
            self.redraw()
            return
        self.editor.set_face(obstacle.obstacle_id, image_face)
        self.recompute_routes_after_edit()
        self.selected_obstacle_id = obstacle.obstacle_id
        self.redraw()

    def delete_selected_obstacle(self, _event: Optional[object] = None) -> None:
        if self.selected_obstacle_id is None:
            return
        if self.editor.delete(self.selected_obstacle_id):
            self.selected_obstacle_id = None
            self.recompute_routes_after_edit()
            self.redraw()

    def recompute_routes_after_edit(self) -> None:
        self.stop_demo()
        self.stop_planned_route()
        self.stop_physical_route()
        edited_obstacles = list(self.editor.obstacles)
        assessment_ready = len(edited_obstacles) == 5 and all(
            obstacle.image_face in IMAGE_FACE_NAMES
            and face_is_available(obstacle, obstacle.image_face, self.planner_config)
            for obstacle in edited_obstacles
        )
        new_route_engine = RouteEngine(self.start_pose, [])
        new_b3_result = B3RouteResult()
        self.obstacles = edited_obstacles
        self.assessment_ready = assessment_ready
        self.route_engine = new_route_engine
        self.b3_result = new_b3_result
        self.route_order = self.route_engine.route_order
        self.route_targets = self.route_engine.route_targets
        self.route_distance = self.route_engine.total_distance
        self.route_order_map = {
            obstacle_id: index + 1 for index, obstacle_id in enumerate(self.route_order)}
        self.route_index = 0
        self.visited_targets = set()
        self.completed_targets = 0
        self.demo_index = 0
        self.demo_remaining = 0.0
        self.pose = Pose(self.start_pose.x, self.start_pose.y,
                         self.start_pose.theta)
        self.plan_succeeded = False
        self.assessment_active = False
        self.active_mode = None
        self.animation_started_at = None
        self.animation_elapsed_s = 0.0
        self.route_paused = False
        self.b2_result = B3RouteResult()
        self.b2_comparison_time_s = None
        self.physical_path = []
        self.physical_path_index = 0
        self.physical_target_indices = []
        self.operator_status = self.placement_status()
        self.edit_error = ""

    def placement_status(self) -> str:
        count = len(self.editor.obstacles) if hasattr(
            self, "editor") else len(self.obstacles)
        if count == 5 and not self.map_is_assessment_ready():
            return "Select an image face for every obstacle before running B2 or B3."
        if count == 5:
            return "Map ready. Choose Run B2 or Run B3 shortest-time."
        return f"Place obstacle {count + 1} of 5"

    def start_assessment(self, _event: Optional[object] = None) -> None:
        self.run_b3(_event)

    def run_b2(self, _event: Optional[object] = None) -> None:
        self.start_route_mode("B2")

    def run_b3(self, _event: Optional[object] = None) -> None:
        self.start_route_mode("B3")

    def start_route_mode(self, mode: str) -> None:
        if not self.map_is_assessment_ready():
            self.operator_status = self.placement_status()
            self.redraw()
            return
        self.stop_demo()
        self.stop_planned_route()
        self.stop_physical_route()
        self.active_mode = mode
        self.operator_status = (
            "Planning B2 continuous route…"
            if mode == "B2"
            else "Evaluating all 120 B3 visit orders…"
        )
        self.edit_error = ""
        self.plan_succeeded = False
        self.assessment_active = True
        self.animation_started_at = None
        self.animation_elapsed_s = 0.0
        self.route_paused = False
        self.redraw()
        self.root.after(20, self.calculate_and_run_b2 if mode ==
                        "B2" else self.calculate_and_run_b3)

    def calculate_and_run_b2(self) -> None:
        result = find_b2_route(
            self.start_pose,
            self.obstacles,
            self.planner_config,
            self.car_planner,
            allow_alternative_viewing=True,
        )
        self.b2_result = result
        self.begin_route_animation(result, "B2")

    def calculate_and_run_b3(self) -> None:
        result = optimise_b3_route(
            self.start_pose,
            self.obstacles,
            self.planner_config,
            self.car_planner,
            allow_alternative_viewing=True,
        )
        self.begin_route_animation(result, "B3")

    def calculate_and_run_assessment(self) -> None:
        self.calculate_and_run_b3()

    def begin_route_animation(self, result: B3RouteResult, mode: str) -> None:
        continuous = route_is_continuous(
            result.leg_results, self.planner_config)
        current = self.start_pose
        endpoints_valid = (
            result.is_valid
            and bool(result.leg_results)
            and len(result.leg_results) == len(result.target_poses)
            and len(result.leg_results) == len(result.obstacle_order)
        )
        if endpoints_valid:
            for leg, target in zip(result.leg_results, result.target_poses):
                if not leg.poses or leg.poses[0] != current or leg.poses[-1] != target:
                    endpoints_valid = False
                    break
                current = target
        if not continuous or not endpoints_valid:
            self.b3_result = result
            self.operator_status = f"No executable {mode} route exists for this map."
            self.edit_error = result.failure_reason or "Planner output failed continuous-path validation."
            self.assessment_active = False
            self.redraw()
            return

        self.b3_result = result
        self.active_mode = mode
        self.route_order = list(result.obstacle_order)
        self.route_targets = list(result.target_poses)
        self.route_order_map = {
            obstacle_id: index + 1 for index, obstacle_id in enumerate(self.route_order)}
        self.physical_path = []
        self.physical_target_indices = []
        for leg in result.leg_results:
            self.physical_path.extend(
                leg.poses if not self.physical_path else leg.poses[1:])
            self.physical_target_indices.append(len(self.physical_path) - 1)
        self.physical_path_index = 0
        self.pose = self.physical_path[0]
        self.visited_targets = set()
        self.completed_targets = 0
        self.plan_succeeded = True
        self.animation_started_at = time.monotonic()
        self.animation_elapsed_s = 0.0
        self.route_paused = False
        route_label = "partial maximum-coverage" if not result.is_complete else "route"
        self.operator_status = f"Running {mode} {route_label} — Completed: 0/5"
        self.redraw()
        self.physical_after_id = self.root.after(
            30, self.animate_physical_route)

    def animate_physical_route(self) -> None:
        self.physical_after_id = None
        if self.route_paused:
            return
        if self.physical_path_index + 1 >= len(self.physical_path):
            self.completed_targets = len(self.route_targets)
            if self.animation_started_at is not None:
                self.animation_elapsed_s = time.monotonic() - self.animation_started_at
            if self.active_mode == "B2":
                self.b2_comparison_time_s = self.animation_elapsed_s
            if self.b3_result.is_complete:
                self.operator_status = f"{self.active_mode} complete — Completed: 5/5."
            else:
                self.operator_status = (
                    f"{self.active_mode} partial route complete — Completed: "
                    f"{self.completed_targets}/5; skipped "
                    f"{self.b3_result.skipped_obstacle_ids}."
                )
            self.assessment_active = False
            self.redraw()
            return
        self.physical_path_index += 1
        self.pose = self.physical_path[self.physical_path_index]
        while (
            self.completed_targets < len(self.physical_target_indices)
            and self.physical_path_index >= self.physical_target_indices[self.completed_targets]
        ):
            self.visited_targets.add(self.route_order[self.completed_targets])
            self.completed_targets += 1
        if self.animation_started_at is not None:
            self.animation_elapsed_s = time.monotonic() - self.animation_started_at
        self.operator_status = (
            f"Running {self.active_mode} route — Completed: {self.completed_targets}/5"
        )
        self.redraw()
        self.physical_after_id = self.root.after(
            30, self.animate_physical_route)

    def toggle_route_pause(self) -> None:
        if not self.plan_succeeded or self.physical_path_index + 1 >= len(self.physical_path):
            return
        if self.route_paused:
            self.route_paused = False
            self.assessment_active = True
            self.animation_started_at = time.monotonic() - self.animation_elapsed_s
            self.operator_status = (
                f"Running {self.active_mode} route — Completed: {self.completed_targets}/5"
            )
            self.physical_after_id = self.root.after(
                30, self.animate_physical_route)
        else:
            if self.animation_started_at is not None:
                self.animation_elapsed_s = time.monotonic() - self.animation_started_at
            self.route_paused = True
            self.stop_physical_route()
            self.operator_status = (
                f"{self.active_mode} paused — Completed: {self.completed_targets}/5"
            )
        self.redraw()

    def _sync_progress_to_physical_index(self) -> None:
        self.pose = self.physical_path[self.physical_path_index]
        self.completed_targets = sum(
            target_index <= self.physical_path_index
            for target_index in self.physical_target_indices
        )
        self.visited_targets = set(self.route_order[:self.completed_targets])

    def previous_route_step(self) -> None:
        if not self.plan_succeeded or self.physical_path_index <= 0:
            return
        if not self.route_paused:
            if self.animation_started_at is not None and self.assessment_active:
                self.animation_elapsed_s = time.monotonic() - self.animation_started_at
            self.route_paused = True
            self.stop_physical_route()
        self.physical_path_index -= 1
        self.assessment_active = True
        self._sync_progress_to_physical_index()
        self.operator_status = (
            f"{self.active_mode} paused — Completed: {self.completed_targets}/5"
        )
        self.redraw()

    def next_route_step(self) -> None:
        if (
            not self.plan_succeeded
            or self.physical_path_index + 1 >= len(self.physical_path)
        ):
            return
        if not self.route_paused:
            self.toggle_route_pause()
        self.physical_path_index += 1
        self._sync_progress_to_physical_index()
        if self.physical_path_index + 1 >= len(self.physical_path):
            self.assessment_active = False
            self.operator_status = (
                f"{self.active_mode} complete — Completed: {self.completed_targets}/5."
            )
            if self.active_mode == "B2":
                self.b2_comparison_time_s = self.animation_elapsed_s
        else:
            self.operator_status = (
                f"{self.active_mode} paused — Completed: {self.completed_targets}/5"
            )
        self.redraw()

    def stop_physical_route(self) -> None:
        if self.physical_after_id is not None:
            self.root.after_cancel(self.physical_after_id)
            self.physical_after_id = None

    def reset_map(self) -> None:
        self.stop_demo()
        self.stop_planned_route()
        self.stop_physical_route()
        self.editor = MapEditorModel([])
        self.obstacles = []
        self.selected_obstacle_id = None
        self.dragged_obstacle_id = None
        self.recompute_routes_after_edit()
        self.redraw()

    def is_valid_pose(self, pose: Pose) -> bool:
        return pose_is_safe(pose, self.obstacles, self.planner_config)

    def manual_straight(self, distance: float) -> None:
        self.stop_demo()
        self.stop_planned_route()
        candidate = move_straight(self.pose, distance)
        if self.is_valid_pose(candidate):
            self.pose = candidate
        self.redraw()

    def manual_curve(self, angle: float) -> None:
        self.stop_demo()
        self.stop_planned_route()
        candidate = move_curve(self.pose, angle)
        if self.is_valid_pose(candidate):
            self.pose = candidate
        self.redraw()

    def stop_demo(self) -> None:
        self.demo_running = False
        if self.demo_after_id is not None:
            self.root.after_cancel(self.demo_after_id)
            self.demo_after_id = None

    def stop_planned_route(self) -> None:
        self.route_engine.running = False
        self.route_running = False
        if self.route_after_id is not None:
            self.root.after_cancel(self.route_after_id)
            self.route_after_id = None

    def reset(self, _event: Optional[object] = None) -> None:
        self.stop_demo()
        self.stop_planned_route()
        self.demo_index = 0
        self.demo_remaining = 0.0
        self.route_engine.reset()
        self.route_index = self.route_engine.route_index
        self.visited_targets = set(self.route_engine.visited_ids)
        self.completed_targets = self.route_engine.completed_targets
        self.pose = self.route_engine.pose
        self.redraw()

    def toggle_demo(self, _event: Optional[object] = None) -> None:
        if self.demo_running:
            self.stop_demo()
            return

        self.stop_planned_route()
        if self.demo_index >= len(self.demo_commands):
            self.demo_index = 0
            self.demo_remaining = 0.0
            self.pose = Pose(self.start_pose.x,
                             self.start_pose.y, self.start_pose.theta)
        self.demo_running = True
        self.demo_after_id = self.root.after(20, self.animate_demo)

    def animate_demo(self) -> None:
        self.demo_after_id = None
        if not self.demo_running:
            return
        if self.demo_index >= len(self.demo_commands):
            self.demo_running = False
            return

        command, total = self.demo_commands[self.demo_index]
        if self.demo_remaining == 0:
            self.demo_remaining = (
                abs(total)
                if command in ("F", "B")
                else abs(total) * TURN_RADIUS_CM
            )

        step = min(ANIMATION_STEP_CM, self.demo_remaining)
        if command == "F":
            candidate = move_straight(self.pose, step)
        elif command == "B":
            candidate = move_straight(self.pose, -step)
        elif command == "L":
            candidate = move_curve(self.pose, step / TURN_RADIUS_CM)
        elif command == "R":
            candidate = move_curve(self.pose, -step / TURN_RADIUS_CM)
        else:
            candidate = self.pose

        if self.is_valid_pose(candidate):
            self.pose = candidate

        self.demo_remaining -= step
        if self.demo_remaining <= 1e-6:
            self.demo_remaining = 0
            self.demo_index += 1
        self.redraw()
        self.demo_after_id = self.root.after(20, self.animate_demo)

    def toggle_planned_route(self, _event: Optional[object] = None) -> None:
        if not self.assessment_ready or not self.route_engine.route_targets:
            self.edit_error = "B2/B3 assessment requires exactly 5 obstacles."
            self.redraw()
            return
        self.stop_demo()
        self.route_engine.toggle()
        self.route_running = self.route_engine.running and not self.route_engine.paused
        if self.route_after_id is not None:
            self.root.after_cancel(self.route_after_id)
            self.route_after_id = None
        if self.route_running:
            self.route_after_id = self.root.after(
                30, self.animate_planned_route)

    def animate_planned_route(self) -> None:
        self.route_after_id = None
        if not self.route_engine.running:
            self.route_running = False
            return
        previous_completed = self.route_engine.completed_targets
        self.pose = self.route_engine.step()
        self.route_index = self.route_engine.route_index
        self.route_running = self.route_engine.running
        self.visited_targets = set(self.route_engine.visited_ids)
        self.completed_targets = self.route_engine.completed_targets
        if self.completed_targets > previous_completed:
            target_id = self.route_engine.visited_ids[-1]
            print(
                f"Reached obstacle {target_id}; completed {self.completed_targets}/{len(self.route_targets)}")
        self.redraw()
        if not self.route_engine.running:
            return
        self.route_after_id = self.root.after(30, self.animate_planned_route)

    def run(self) -> None:
        self.root.mainloop()
