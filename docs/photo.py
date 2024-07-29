from PIL import Image
import os

def resize_image(img, height):
    """将图片按高度等比例缩放"""
    aspect_ratio = img.width / img.height
    new_width = int(height * aspect_ratio)
    return img.resize((new_width, height))

def merge_images(image_paths, output_path, new_height):
    """将多张图片调整为相同高度并水平合并"""
    images = []
    for img_path in image_paths:
        if os.path.exists(img_path):
            print(f"正在打开图片: {img_path}")
            images.append(Image.open(img_path))
        else:
            print(f"找不到图片: {img_path}")
            return

    # 调整每张图片的大小
    resized_images = [resize_image(img, new_height) for img in images]

    # 获取每个图片的宽度
    widths, heights = zip(*(img.size for img in resized_images))

    # 计算合并后图片的总宽度
    total_width = sum(widths)

    # 创建一个新的空白图片，大小为合并后图片的总宽度和统一高度
    new_img = Image.new('RGB', (total_width, new_height))

    # 将图片粘贴到新的空白图片中
    x_offset = 0
    for img in resized_images:
        new_img.paste(img, (x_offset, 0))
        x_offset += img.width

    # 保存合并后的图片
    new_img.save(output_path)
    print(f"图片合并完成，保存为 {output_path}")

if __name__ == "__main__":
    # 图片文件路径
    image_paths = [
        "/home/lsy/manipulator_control/src/docs/GripperGazebo.png",
        "/home/lsy/manipulator_control/src/docs/GripperReal.jpg",
        "/home/lsy/manipulator_control/src/docs/GripperRviz.png"
    ]

    # 合并后图片的保存路径
    output_path = "GripperCombined.png"

    # 设定新的统一高度
    new_height = 400  # 根据需要调整这个值

    # 执行合并操作
    merge_images(image_paths, output_path, new_height)
