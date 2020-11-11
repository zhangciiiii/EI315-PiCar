# -*- coding: utf-8 -*-
"""
Created on Fri Jun 22 12:16:45 2018

@author: Dynasting
"""


import requests,base64

def get_acccess_token():

    host = 'https://aip.baidubce.com/oauth/2.0/token?grant_type=client_credentials&client_id=huHitRbbUZPpTLGCoUSIBwSX&client_secret=7Fzs3tGkrmgPxySTCfre0lCocdgFIhBN&'
    
    header_1 = {'Content-Type':'application/json; charset=UTF-8'}
    
    request=requests.post(host,headers =header_1)
    
    access_token=request.json()['access_token']
    return access_token


access_token = get_acccess_token()




#def call_api(access_token):
    
url = 'https://aip.baidubce.com/rest/2.0/ocr/v1/general_basic?access_token=' + access_token

f = open(r'OCR1.jpg', 'rb')
# 参数image：图像base64编码
img = base64.b64encode(f.read())

header_2 = {'Content-Type':'application/x-www-form-urlencoded'}
params = {"image": img}


rqst=requests.post(url,params=params,headers=header_2)

for words in rqst.json()['words_result']:
    print(words['words'])