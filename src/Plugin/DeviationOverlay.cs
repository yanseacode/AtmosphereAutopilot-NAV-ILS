using System;
using AtmosphereAutopilotNav.Core;
using KSP.UI;
using KSP.UI.Screens.Flight;
using UnityEngine;

namespace AtmosphereAutopilotNav.Plugin
{
    // Independent renderer matching the observable Kramax deviation display.
    // It only reads guidance; displaying it cannot alter aircraft control or leg sequencing.
    internal sealed class DeviationOverlay
    {
        private NavBall navball;
        private GUIStyle labelStyle;
        private static Texture2D diamond;
        private Vector2 positionOffset;
        private Vector2 dragStartMouse;
        private Vector2 dragStartOffset;
        private int dragControl;

        public void CancelDrag()
        {
            if (dragControl != 0 && GUIUtility.hotControl == dragControl)
                GUIUtility.hotControl = 0;
            dragControl = 0;
        }

        public void Draw(bool horizontal, bool vertical, double crossTrack, double altitudeError)
        {
            if (!horizontal && !vertical) { CancelDrag(); return; }
            if (navball == null) navball = UnityEngine.Object.FindObjectOfType<NavBall>();
            if (navball == null || !navball.gameObject.activeInHierarchy || UIMasterController.Instance == null)
            { CancelDrag(); return; }
            var canvas = UIMasterController.Instance.appCanvas;
            if (canvas == null || canvas.worldCamera == null) { CancelDrag(); return; }
            Vector3 center = canvas.worldCamera.WorldToScreenPoint(navball.transform.position);
            if (center.y <= 0 || center.z < 0) { CancelDrag(); return; }

            // Same reference geometry as Kramax, anchored to the actual navball camera position.
            float width = (int)(center.y * 300f / 85.6f);
            float height = (int)(center.y * 255f / 85.6f);
            float x = (int)(0.5f + center.x - center.y * 223f / 171.2f - 33f * width / 300f);
            float y = (int)(0.5f + Screen.height - center.y * 200f / 85.6f - 40f * height / 255f);
            if (width <= 60 || height <= 80) { CancelDrag(); return; }

            // Capture only the visible bars, leaving the navball and the space between them clickable.
            int control = GUIUtility.GetControlID(88439963, FocusType.Passive);
            Event input = Event.current;
            Vector2 mouse = GUIUtility.GUIToScreenPoint(input.mousePosition);
            Rect horizontalHit = new Rect(x + positionOffset.x + 20, y + positionOffset.y + 20, width - 60, 20);
            Rect verticalHit = new Rect(x + positionOffset.x + width - 40, y + positionOffset.y + 40, 20, height - 80);
            EventType eventType = input.GetTypeForControl(control);
            if (eventType == EventType.MouseDown && input.button == 0 && GUIUtility.hotControl == 0 &&
                ((horizontal && horizontalHit.Contains(mouse)) || (vertical && verticalHit.Contains(mouse))))
            {
                GUIUtility.hotControl = control;
                dragControl = control;
                dragStartMouse = mouse;
                dragStartOffset = positionOffset;
                input.Use();
            }
            else if (GUIUtility.hotControl == control && dragControl == control)
            {
                if (eventType == EventType.MouseDrag)
                {
                    positionOffset = dragStartOffset + mouse - dragStartMouse;
                    input.Use();
                }
                else if (eventType == EventType.MouseUp && input.button == 0)
                {
                    CancelDrag();
                    input.Use();
                }
            }
            if (input.type != EventType.Repaint) return;
            x += positionOffset.x;
            y += positionOffset.y;

            if (labelStyle == null)
            {
                labelStyle = new GUIStyle(GUI.skin.label) { alignment = TextAnchor.MiddleCenter, fontSize = 12 };
                labelStyle.normal.textColor = XKCDColors.NeonGreen;
            }
            Color background = GUI.backgroundColor;
            Color foreground = GUI.color;
            Matrix4x4 matrix = GUI.matrix;
            try
            {
                // Coordinates above are physical screen pixels, even when AA scales its windows.
                GUI.matrix = Matrix4x4.identity;
                GUI.color = Color.white;
                GUI.backgroundColor = XKCDColors.NeonRed;
                if (horizontal)
                {
                    Rect bar = new Rect(x + 20, y + 20, width - 60, 20);
                    DrawBar(bar, false, crossTrack);
                    GUI.Label(new Rect(bar.x, y, bar.width, 20), DeviationDisplay.Label(crossTrack), labelStyle);
                }
                if (vertical)
                {
                    Rect bar = new Rect(x + width - 40, y + 40, 20, height - 80);
                    DrawBar(bar, true, altitudeError);
                    GUI.Label(new Rect(bar.center.x - 47, bar.yMax, 90, 20),
                        DeviationDisplay.Label(altitudeError), labelStyle);
                }
            }
            finally
            {
                GUI.matrix = matrix;
                GUI.backgroundColor = background;
                GUI.color = foreground;
            }
        }

        private static void DrawBar(Rect bounds, bool vertical, double meters)
        {
            GUI.Box(bounds, GUIContent.none);
            float length = vertical ? bounds.height : bounds.width;
            float center = Mathf.Floor(length * 0.5f + 0.5f);
            float step = Math.Max(1, Mathf.Floor(length * 0.11f + 0.5f));
            for (int i = -4; i <= 4; i++)
            {
                float position = center + i * step;
                if (position <= 0 || position >= length) continue;
                bool middle = i == 0;
                float inset = middle ? 1 : 3;
                float thickness = middle ? 3 : 1;
                float tickPosition = position - (middle ? 1 : 0);
                Rect tick = vertical
                    ? new Rect(bounds.x + inset, bounds.y + tickPosition, bounds.width - (middle ? 3 : 6), thickness)
                    : new Rect(bounds.x + tickPosition, bounds.y + inset, thickness, bounds.height - (middle ? 3 : 6));
                GUI.color = middle ? XKCDColors.NeonBlue : XKCDColors.NeonGreen;
                GUI.DrawTexture(tick, Texture2D.whiteTexture);
            }
            if (NavigationMath.Finite(meters))
            {
                float offset = (float)DeviationDisplay.Deflection(meters) * (vertical ? 1 : -1);
                float position = center + (length - 10) * offset * 0.5f - 4;
                Rect marker = vertical
                    ? new Rect(bounds.x + 3, bounds.y + position, 15, 9)
                    : new Rect(bounds.x + position, bounds.y + 3, 9, 15);
                GUI.color = Math.Abs(meters) > 100 ? XKCDColors.NeonRed : XKCDColors.NeonBlue;
                GUI.DrawTexture(marker, Diamond());
            }
            GUI.color = Color.white;
        }

        private static Texture2D Diamond()
        {
            if (diamond != null) return diamond;
            diamond = new Texture2D(19, 19, TextureFormat.ARGB32, false);
            diamond.hideFlags = HideFlags.HideAndDontSave;
            for (int y = 0; y < 19; y++)
                for (int x = 0; x < 19; x++)
                    diamond.SetPixel(x, y, Math.Abs(x - 9) + Math.Abs(y - 9) <= 9 ? Color.white : Color.clear);
            diamond.Apply();
            return diamond;
        }
    }
}
