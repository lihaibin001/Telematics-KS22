import urllib
import urllib.request
import re


def get_html(url):
    page = urllib.request.urlopen(url)
    html_tmp = page.read()
    return html_tmp


def get_image(argument):
    reg = r'src="(.+?\.jpg)"'
    image_re = re.compile(reg)
    img_list = re.findall(image_re, str(argument))
    x = 0
    for img_url in img_list:
        try:
            f = urllib.request.urlretrieve(img_url, "%d.jpg" % x)
        except urllib.error.HTTPError as e:
            print("Error: ", e)
        x += 1
    return img_list

html = get_html("https://tieba.baidu.com/f?kw=nba")
print(get_image(html))
