import xml.etree.ElementTree as ET

class MapLoader:
    def __init__(self, file_path, safety_margin=0.5):
        self.file_path = file_path
        self.safety_margin = safety_margin  # Biên an toàn

    def load_obstacles(self):
        """Parse .world (SDF) file và lấy danh sách obstacles với kích thước đúng."""
        tree = ET.parse(self.file_path)
        root = tree.getroot()

        obstacles = []

        # Tìm tất cả các thẻ <model> trong world
        for model in root.findall(".//model"):
            model_name = model.get("name", "unknown")
            pose = model.find("pose")
            box_size = model.find(".//geometry/box/size")

            if pose is not None and box_size is not None:
                # Lấy vị trí (x, y, z)
                x, y, z, *_ = map(float, pose.text.split())

                # Lấy kích thước (dx, dy, dz)
                dx, dy, dz = map(float, box_size.text.split())

                # Tính toán vùng bị cấm (thêm biên an toàn)
                obstacles.append({
                    "name": model_name,
                    "x_min": x - dx / 2 - self.safety_margin,
                    "x_max": x + dx / 2 + self.safety_margin,
                    "y_min": y - dy / 2 - self.safety_margin,
                    "y_max": y + dy / 2 + self.safety_margin,
                    "z_min": z - dz / 2 - self.safety_margin,
                    "z_max": z + dz / 2 + self.safety_margin,
                })

        return obstacles
