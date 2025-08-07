import cv2
import pytesseract
from PIL import Image

def recognize_text_simple(image_path):
    """
    简单的文字识别函数，使用默认语言包
    
    参数:
        image_path: 图片路径
        
    返回:
        识别出的文字字符串
    """
    try:
        # 读取图像
        img = cv2.imread(image_path)
        
        # 转换为灰度图
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # 增强对比度
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        enhanced = clahe.apply(gray)
        
        # 二值化
        _, binary = cv2.threshold(enhanced, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        
        # 保存预处理后的图像，便于查看效果
        cv2.imwrite('preprocessed.jpg', binary)
        
        # 转为PIL图像
        pil_img = Image.fromarray(binary)
        
        # 使用默认语言包
        text = pytesseract.image_to_string(pil_img)
        
        return text
    except Exception as e:
        return f"错误: {str(e)}"

# 检查Tesseract安装和版本
def check_tesseract():
    try:
        # 获取Tesseract版本
        version = pytesseract.get_tesseract_version()
        print(f"Tesseract版本: {version}")
        
        # 检查可用语言
        langs = pytesseract.get_languages()
        print(f"可用语言包: {langs}")
        
        return True
    except Exception as e:
        print(f"Tesseract检查错误: {str(e)}")
        return False

# 设置Tesseract路径（如果需要）
def set_tesseract_path(path="C:\\Program Files\\Tesseract-OCR\\tesseract.exe"):
    """
    设置Tesseract可执行文件路径
    如果Tesseract添加到了PATH环境变量，则不需要调用此函数
    """
    pytesseract.pytesseract.tesseract_cmd = path

# 使用示例
if __name__ == "__main__":
    # 可选：设置Tesseract路径
    set_tesseract_path()
    
    # 检查Tesseract安装
    check_tesseract()
    
    # 识别文字
    image_path = 'ocr_test.jpg'  # 替换为你的图片路径
    result = recognize_text_simple(image_path)
    
    print("\n识别结果:")
    print(result)