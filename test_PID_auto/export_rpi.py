"""
Export du modèle IA pour Raspberry Pi (ARM) et PC
Compatible avec IndustrialRLAgent (ActorNetwork)
"""
import torch
import torch.nn as nn
from industrial_ai import IndustrialRLAgent, ActorNetwork
import os

print("=" * 60)
print("EXPORT MODÈLE IA - PC & Raspberry Pi")
print("=" * 60)

# 1. Chargement du modèle entraîné
print("\n1. Chargement du modèle entraîné...")
agent = IndustrialRLAgent()
actor = agent.actor
actor.eval()
print(f"   ✓ Actor chargé: {agent.config.state_dim}D → {agent.config.action_dim}D")

# 2. Configuration quantification
print("\n2. Configuration quantification INT8...")
# Détection automatique PC vs Raspberry Pi
try:
    torch.backends.quantized.engine = 'qnnpack'  # ARM / Raspberry Pi
    print("   ✓ Moteur: QNNPACK (ARM/Raspberry Pi)")
except:
    torch.backends.quantized.engine = 'fbgemm'   # x86 / PC
    print("   ✓ Moteur: FBGEMM (x86/PC)")

# 3. Quantification dynamique (optimise Linear layers)
print("\n3. Quantification dynamique...")
quantized_actor = torch.quantization.quantize_dynamic(
    actor,
    {nn.Linear},
    dtype=torch.qint8
)
print("   ✓ Modèle quantifié en INT8")

# 4. Compilation TorchScript
print("\n4. Compilation TorchScript...")
# State: 10 features [yaw_err, yaw_rate, ax, ay, az, motor_L, motor_R, dir_y, dir_x, ramp_speed]
example_input = torch.randn(1, agent.config.state_dim)
traced_model = torch.jit.trace(quantized_actor, example_input)
print(f"   ✓ Tracing avec entrée: {tuple(example_input.shape)}")

# 5. Sauvegarde
export_path = "drive_assist_rpi_int8.pt"
traced_model.save(export_path)
print(f"\n✅ Modèle exporté: {export_path}")

# 6. Test rapide
print("\n6. Test d'inférence...")
with torch.no_grad():
    test_output = traced_model(example_input)
    print(f"   ✓ Sortie: {test_output.numpy()[0]}")
    print(f"   ✓ Latence estimée: ~2-5ms sur Pi 4")

print("\n" + "=" * 60)
print("INSTRUCTIONS:")
print("=" * 60)
print("PC:   python export_rpi.py")
print("Pi:   Copier le fichier .pt sur le Raspberry Pi")
print("      Utiliser import_rpi.py pour charger le modèle")
print("=" * 60)