import tkinter as tk
from PIL import Image, ImageTk
import quadruped

class TargetUI:
    def __init__(self, neutral_positions):
        """
        Input: 
            neutral_positions: list of 4 (x, y, z) positions in order of Quadruped._GROUPS
        """
        self.root = tk.Tk()
        self.viewonly = False
        self.lock_sensible_axis = False # Locks non-mirrored axis together
        self.root.title("Foot Targets")

        # Set up grid frames
        main_frame = tk.Frame(self.root)
        main_frame.pack(padx=10, pady=10)

        left_column = tk.Frame(main_frame)
        mid_column = tk.Frame(main_frame)
        right_column = tk.Frame(main_frame)
        ctl_column = tk.Frame(main_frame)

        left_column.grid (row=0, column=0, sticky="ns")
        mid_column.grid  (row=0, column=1, padx=10, sticky="ns")
        right_column.grid(row=0, column=2, sticky="ns")
        ctl_column.grid(row=0, column=3, sticky="ns")

        # Janky solution to remap each leg to where it should be in the image
        _placement = {
            3: (left_column,  0),   # Front Left  — top-left
            2: (left_column,  1),   # Back Left   — bottom-left
            0: (right_column, 0),   # Front Right — top-right
            1: (right_column, 1),   # Back Right  — bottom-right
        }

        # Create all slider groups
        self.vars = []
        for i, (x0, y0, z0) in enumerate(neutral_positions):
            column_frame, row = _placement[i]
            frame = tk.LabelFrame(column_frame, text=quadruped.Quadruped._GROUPS[i], padx=5, pady=5)
            frame.grid(row=row, column=0, padx=5, pady=5, sticky="ew")

            x = tk.DoubleVar(value=x0)
            y = tk.DoubleVar(value=y0)
            z = tk.DoubleVar(value=z0)
            self.vars.append((x, y, z))
            
            # self._make_slider(frame, "x (forward)",  x, -0.15, 0.15)
            # self._make_slider(frame, "y (lateral)",  y,  0.02, 0.15)
            # self._make_slider(frame, "z (vertical)", z, -0.31, -0.10)
            # TESTING LIMITS UNTIL ERROR
            self._make_slider(frame, "x (forward)",  x, -0.221,  0.221)
            self._make_slider(frame, "y (lateral)",  y,  -0.225, 0.225)
            self._make_slider(frame, "z (vertical)", z, -0.30,  -0.1)

        # Create image
        self.root.update_idletasks()
        col_height = left_column.winfo_reqheight()
        img = Image.open("ui_images/quad_top_view.png")
        aspect = img.width / img.height
        img = img.resize((int(col_height * aspect), col_height), Image.LANCZOS)
        self._photo = ImageTk.PhotoImage(img)
        tk.Label(mid_column, image=self._photo).pack(expand=True)

    def _make_slider(self, parent, label, var, from_, to):
        tk.Label(parent, text=label).pack()
        tk.Scale(parent, variable=var, from_=from_, to=to,
                 resolution=0.001, orient=tk.HORIZONTAL,
                 length=300).pack()

    def get_targets(self):
        if not self.lock_sensible_axis:
            return [(x.get(), y.get(), z.get()) for x, y, z in self.vars]
        else:
            zx = self.vars[0][0].get()
            zt = self.vars[0][2].get()
            return [(zx, y.get(), zt) for x, y, z in self.vars] 
    
    # Set the slider values based off of target_tuples
    def set_targets(self, targets, lock_height):
        for i, (x, y, z) in enumerate(targets):
            self.vars[i][0].set(x)
            self.vars[i][1].set(y)
            self.vars[i][2].set(z)

    def update(self):
        self.root.update()

    def close(self):
        self.root.destroy()