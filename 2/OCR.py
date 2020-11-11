# -*- coding: utf-8 -*-
"""
Created on Fri Jun 22 11:56:17 2018

@author: Dynasting
"""

from aip import AipOcr
import cv2
APP_ID = '11430256'
API_KEY = 'huHitRbbUZPpTLGCoUSIBwSX'
SECRET_KEY = '7Fzs3tGkrmgPxySTCfre0lCocdgFIhBN '

client = AipOcr(APP_ID, API_KEY, SECRET_KEY)
# =============================================================================
# """take photo"""
# 
# cap = cv2.VideoCapture(0)
# 
# cv2.waitKey(3000)
# 
# ret,frame = cap.read()
# 
# cv2.imwrite("OCR1.jpg",frame)
# 
# =============================================================================

""" 读取图片 """
def get_file_content(filePath):
    with open(filePath, 'rb') as fp:
        return fp.read()

image = get_file_content('OCR1.jpg')

""" 调用通用文字识别, 图片参数为本地图片 """
client.basicGeneral(image);

""" 如果有可选参数 """
options = {}
options["language_type"] = "CHN_ENG"
options["detect_direction"] = "true"
options["detect_language"] = "true"
options["probability"] = "true"

""" 带参数调用通用文字识别, 图片参数为本地图片 """
res = client.basicGeneral(image, options)

for w in res['words_result']:
    print(w['words'])    