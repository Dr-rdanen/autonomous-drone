import cv2 as cv
import numpy as np

cap = cv.VideoCapture(0)
cap.set(3, 640)     #set ukuran window
cap.set(4, 480)     #set ukuran window

R = 1                               #deklarasi variable sebagai syarat deteksi warna red
G = 1                               #deklarasi variable sebagai syarat deteksi warna green
B = 1                               #deklarasi variable sebagai syarat deteksi warna blue
P = 1                               #deklarasi variable sebagai syarat deteksi warna purple
areamin = 35                #set area minimal deteksi contour

focal_length = 303.8071         #focal length camera, tergantung camera yang digunakan
lebar = 10                      #lebar benda di dunia nyata

#colour_name = ["BLUE", "PURPLE", "GREEN", "RED"]
colour_box = [(255,0,0),(255,0,255),(0,255,0), (0,0,255)]                   #array warna kotak saat mendeteksi
colour_up = [(122,255,255),(160, 255, 255),(75,255,255), (5,255,255)]       #array batas atas warna-warna
colour_low = [(93,51,51),(125, 51, 51), (50,51,51), (0,51,51)]              #array batas bawah warna-warna

def jarak(lebar_asli, lebar_pixel, focal):                  #fungsi kalkulasi jarak benda
    distance = (lebar_asli * focal)/(lebar_pixel*0.62)      #ngitung
    return distance                                         #balikin nilainya

class warna:
    def __init__(self, colour):                     #mengambil data untuk objek dengan variable colour (0 sampai 3)
        self.colour = colour                        #membuat variable self.colour = argumen colour yang dimasukkan untuk objek
        self.lower = np.array(colour_low[colour])   #set batas bawah warna dari array colour_low untuk objek tersebut
        self.upper = np.array(colour_up[colour])    #set batas atas warna dari array colour_up untuk objek tersebut
        self.box = np.array(colour_box[colour])     #set warna kotak dari array colour_low untuk objek tersebut

    def mask(self,img):                                     #untuk membuat mask dari warna objek
        blur = cv.GaussianBlur(img, (5, 5), 0)  #blur gambar img
        hsv = cv.cvtColor(blur, cv.COLOR_BGR2HSV)            #mengubah gambar menjadi hsv (hue, saturation, value); yang warna warni itu
        mask = cv.inRange(hsv, self.lower, self.upper)      #membuat mask dari hsv, mencari warna dari batas bawah dan atas objek saja

        kernel = np.ones((25, 25), np.uint8)          #deklarasi kernel untuk pixel contour

        mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, kernel)    #mengurangi noise luar contour  (kalo gasalah)
        mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)     #mengurangi noise dalam contour (kalo gasalah)
        return mask

    def contours (self, img):
        global R,G,B,P,areamin          #ngambil variabel global R,G,B,P,areamin supaya bisa dipakai

        contours, hierarchy = cv.findContours(self.mask(img), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)     #membaut contours dari mask objek yang dibuat
        for cnt in contours:
            big = max(contours, key=cv.contourArea)         #mencari contour yang terbesar diantar semua contour yang terdeteksi (untuk warna objek itu saja)
            x, y, w, h = cv.boundingRect(big)               #mencari titik-titik kotak (kayak hitbox) dari contour
            distance = jarak(lebar, w, focal_length)        #ngitung jarak benda dengan fungsi 'jarak'
            jauh = float(distance)                          #konversi variable jadi float

            if len(big) > areamin:                          #ngecek apa luas contour terbesar > areamin

                if self.colour == 0 and B == 1:             #untuk objek dengan warna biru
                    if x + w < 340 and y + h < 240:         #ngecek apa contour biru terdeteksi di kotak kiri atas
                        cv.rectangle(img, (x, y), (x + w, y + h), colour_box[self.colour], 3)       #kotakin contour biru yang memenuhi syarat di kotak kiri atas
                        if 130 < x + (w / 2) < 170 and 110 < y + (h / 2) < 131 and jauh > 25:     #ngecek tengah tengah contour biru ada di lingkaran putih dan jaraknya > 25
                            B = 0                           #jika iya, buat B=0, jadinya contour biru tidak terdeteksi lagi

                if self.colour == 1 and P == 1:             #untuk objek dengan warna ungu
                    if x > 320 and y + h < 240:             #ngecek apa contour ungu terdeteksi di kotak kanan atas
                        cv.rectangle(img, (x, y), (x + w, y + h), colour_box[self.colour], 3)       #kotakin contour ungu yang memenuhi syarat di kotak kanan atas
                        if 470 < x + (w / 2) < 490 and 110 < y + (h / 2) < 131 and jauh > 25:     #ngecek tengah-tengah contour ungu ada di lingkaran putih dan jaraknya > 25
                            P = 0                           #jika iya, buat P=0, jadinya contour ungu tidak terdeteksi lagi

                if self.colour == 2 and G == 1:             #untuk objek dengan warna hijau
                    if x + w < 320 and y > 240:             #ngecek apa contour hijau terdeteksi di kotak kiri bawah
                        cv.rectangle(img, (x, y), (x + w, y + h), colour_box[self.colour], 3)       #kotakin contour hijau yang memenuhi syarat di kotak kiri bawah
                        if 130 < x + (w / 2) < 170 and 350 < y + (h / 2) < 370 and jauh > 25:     #ngecek tenga-tengah contour hijau ada di lingkaran putih dan jaraknya > 25
                            G = 0                           #jika iya, buat G=0, jadinya contour hijau tidak terdeteksi lagi

                if self.colour == 3 and R == 1:             #untuk objek dengan warna merah
                    if x > 320 and y > 240:                 #ngecek apa contour merah terdeteksi di kotak kanan bawah
                        cv.rectangle(img, (x, y), (x + w, y + h), colour_box[self.colour], 3)       #kotakin contour merah yang memenuhi syarat di kotak kanan bawah
                        if 470 < x + (w / 2) < 490 and 350 < y + (h / 2) < 370 and jauh > 25:     #ngecek tengah-tengah contour merah ada di lingkaran putih dan jaraknya > 25
                            R = 0                           #jika iya, buat R=0, jaidnya contour merah tidak terdeteksi lagi

repeat = True                   #syarat while ngeloop terus
while repeat == True:
    isTrue, img = cap.read()    #membuat img sebagai hasil dari read/bacaan nya camera, deklarasi img sebagai hasil bacaan camera

    colour1 = warna(0)          #buat objek dari class warna, untuk warna biru
    colour2 = warna(1)          #buat objek dari class warna, untuk warna ungu
    colour3 = warna(2)          #buat objek dari class warna, untuk warna hijau
    colour4 = warna(3)          #buat objek dari class warna, untuk warna merah

    cv.line(img, (320, 0), (320,480), (255,255,255), 2)  #bikin garis putih vertical dari atas ke bawah
    cv.line(img, (0, 240), (640,240), (255,255,255), 2)   #bikin garis putih horizontal dari kiri ke kanan

    cv.circle(img, (160, 120), 5, (255, 255, 255), 2)   #bikikn lingkaran putih di kotak kiri atas
    cv.circle(img, (480, 120), 5, (255, 255, 255), 2)   #bikikn lingkaran putih di kotak kanan atas
    cv.circle(img, (160, 360), 5, (255, 255, 255), 2)   #bikikn lingkaran putih di kotak kiri bawah
    cv.circle(img, (480, 360), 5, (255, 255, 255), 2)   #bikikn lingkaran putih di kotak kanan bawah

    cv.putText(img, str(B), (5, 230), cv.FONT_HERSHEY_SIMPLEX, 1, (255,0,0), 2)     #nunjukkin nilai B
    cv.putText(img, str(P), (615, 230), cv.FONT_HERSHEY_SIMPLEX, 1, (255,0,255), 2)     #nunjukkin nilai P
    cv.putText(img, str(G), (5, 270), cv.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)     #nunjukkin nilai G
    cv.putText(img, str(R), (615, 270), cv.FONT_HERSHEY_SIMPLEX, 1, (0,0,255), 2)   #nunjukkin nilai R

    colour1.contours(img)       #untuk jalanin methode contours, untuk warna biru
    colour2.contours(img)       #untuk jalanin methode contours, untuk warna ungu
    colour3.contours(img)       #untuk jalanin methode contours, untuk warna hijau
    colour4.contours(img)       #untuk jalanin methode contours, untuk warna merah

    cv.imshow("image", img)     #nampilin window img

    key = cv.waitKey(1)             #nunggu pencetan
    if key == 27:                   #kalo 'esc' kepencet, loop langsung berhenti
        repeat = False

cap.release()                   #menghentikan camera
cv.destroyAllWindows()          #menutup semua window