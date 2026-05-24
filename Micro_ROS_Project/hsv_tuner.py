#!/usr/bin/env python3
"""
========================================================================
OUTIL DE REGLAGE HSV INTERACTIF
========================================================================
Permet de trouver les bonnes plages HSV pour TES boites sous TON
eclairage. Indispensable avant de figer les valeurs dans le code.

UTILISATION :
  1. Lance ce script
  2. Pointe ta boite (rouge, vert, ou bleu) vers la camera
  3. Bouge les curseurs H/S/V jusqu'a ce que SEULE ta boite soit
     blanche dans la fenetre "Mask"
  4. Note les valeurs affichees dans la console
  5. Reporte-les dans ton code principal

  Clic GAUCHE sur l'image : echantillonne la couleur du pixel
  (auto-remplit les curseurs autour de cette teinte)

CONTROLES :
  Q = quitter
  S = afficher les valeurs HSV actuelles dans la console
  C = clic-pour-echantillonner ON/OFF
========================================================================
"""

import cv2
import numpy as np

CAMERA_INDEX = 1          # mets l'index de ta camera externe
CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480

# Etat global pour l'echantillonnage par clic
sampled_hsv = None


def nothing(x):
    pass


def on_mouse(event, x, y, flags, param):
    """Echantillonne la couleur HSV au clic."""
    global sampled_hsv
    if event == cv2.EVENT_LBUTTONDOWN:
        hsv_frame = param['hsv']
        if hsv_frame is not None and 0 <= y < hsv_frame.shape[0] and 0 <= x < hsv_frame.shape[1]:
            sampled_hsv = hsv_frame[y, x].copy()
            print(f"[ECHANTILLON] Pixel ({x},{y}) -> HSV = "
                  f"H:{sampled_hsv[0]} S:{sampled_hsv[1]} V:{sampled_hsv[2]}")


def main():
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)

    # Fenetre de controle avec trackbars
    cv2.namedWindow("Reglages HSV")
    cv2.createTrackbar("H min", "Reglages HSV", 0, 179, nothing)
    cv2.createTrackbar("H max", "Reglages HSV", 179, 179, nothing)
    cv2.createTrackbar("S min", "Reglages HSV", 0, 255, nothing)
    cv2.createTrackbar("S max", "Reglages HSV", 255, 255, nothing)
    cv2.createTrackbar("V min", "Reglages HSV", 0, 255, nothing)
    cv2.createTrackbar("V max", "Reglages HSV", 255, 255, nothing)

    mouse_param = {'hsv': None}
    cv2.namedWindow("Camera")
    cv2.setMouseCallback("Camera", on_mouse, mouse_param)

    print("=" * 60)
    print("OUTIL DE REGLAGE HSV")
    print("=" * 60)
    print("1. Pointe ta boite coloree vers la camera")
    print("2. CLIQUE sur la boite dans la fenetre 'Camera'")
    print("   (ca remplit les curseurs automatiquement)")
    print("3. Affine avec les curseurs jusqu'a isoler la boite")
    print("4. Appuie 'S' pour afficher les valeurs a copier")
    print("=" * 60)

    global sampled_hsv

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mouse_param['hsv'] = hsv

        # Si un echantillon a ete clique, auto-remplir les curseurs
        if sampled_hsv is not None:
            h, s, v = int(sampled_hsv[0]), int(sampled_hsv[1]), int(sampled_hsv[2])
            # Marge autour de la teinte echantillonnee
            cv2.setTrackbarPos("H min", "Reglages HSV", max(0, h - 10))
            cv2.setTrackbarPos("H max", "Reglages HSV", min(179, h + 10))
            cv2.setTrackbarPos("S min", "Reglages HSV", max(0, s - 60))
            cv2.setTrackbarPos("S max", "Reglages HSV", 255)
            cv2.setTrackbarPos("V min", "Reglages HSV", max(0, v - 60))
            cv2.setTrackbarPos("V max", "Reglages HSV", 255)
            sampled_hsv = None

        # Lire les curseurs
        h_min = cv2.getTrackbarPos("H min", "Reglages HSV")
        h_max = cv2.getTrackbarPos("H max", "Reglages HSV")
        s_min = cv2.getTrackbarPos("S min", "Reglages HSV")
        s_max = cv2.getTrackbarPos("S max", "Reglages HSV")
        v_min = cv2.getTrackbarPos("V min", "Reglages HSV")
        v_max = cv2.getTrackbarPos("V max", "Reglages HSV")

        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])
        mask = cv2.inRange(hsv, lower, upper)

        # Nettoyage (comme dans le code final)
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask_clean = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
        mask_clean = cv2.morphologyEx(mask_clean, cv2.MORPH_CLOSE,
                                      cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7)))

        # Resultat : image masquee
        result = cv2.bitwise_and(frame, frame, mask=mask_clean)

        # Compteur de pixels
        count = cv2.countNonZero(mask_clean)
        cv2.putText(frame, f"Pixels: {count}", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(frame, "Clique sur ta boite", (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        cv2.imshow("Camera", frame)
        cv2.imshow("Mask (ta boite doit etre BLANCHE seule)", mask_clean)
        cv2.imshow("Resultat", result)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            print("\n" + "=" * 50)
            print("VALEURS A COPIER DANS TON CODE :")
            print("=" * 50)
            print(f"COLOR_L = np.array([{h_min}, {s_min}, {v_min}], dtype=np.uint8)")
            print(f"COLOR_U = np.array([{h_max}, {s_max}, {v_max}], dtype=np.uint8)")
            print(f"# Pixels detectes: {count}")
            print("=" * 50 + "\n")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
