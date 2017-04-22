package org.firstinspires.ftc.teamcode;

import android.graphics.Point;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.Collections;

import static org.firstinspires.ftc.teamcode.superText.numbers;

public class tetris {

    private final Point[][][] Tetraminos = {
            // I-Piece
            {
                    {new Point(0, 1), new Point(1, 1), new Point(2, 1), new Point(3, 1)},
                    {new Point(1, 0), new Point(1, 1), new Point(1, 2), new Point(1, 3)},
                    {new Point(0, 1), new Point(1, 1), new Point(2, 1), new Point(3, 1)},
                    {new Point(1, 0), new Point(1, 1), new Point(1, 2), new Point(1, 3)}
            },

            // J-Piece
            {
                    {new Point(0, 1), new Point(1, 1), new Point(2, 1), new Point(2, 0)},
                    {new Point(1, 0), new Point(1, 1), new Point(1, 2), new Point(2, 2)},
                    {new Point(0, 1), new Point(1, 1), new Point(2, 1), new Point(0, 2)},
                    {new Point(1, 0), new Point(1, 1), new Point(1, 2), new Point(0, 0)}
            },

            // L-Piece
            {
                    {new Point(0, 1), new Point(1, 1), new Point(2, 1), new Point(2, 2)},
                    {new Point(1, 0), new Point(1, 1), new Point(1, 2), new Point(0, 2)},
                    {new Point(0, 1), new Point(1, 1), new Point(2, 1), new Point(0, 0)},
                    {new Point(1, 0), new Point(1, 1), new Point(1, 2), new Point(2, 0)}
            },

            // O-Piece
            {
                    {new Point(0, 0), new Point(0, 1), new Point(1, 0), new Point(1, 1)},
                    {new Point(0, 0), new Point(0, 1), new Point(1, 0), new Point(1, 1)},
                    {new Point(0, 0), new Point(0, 1), new Point(1, 0), new Point(1, 1)},
                    {new Point(0, 0), new Point(0, 1), new Point(1, 0), new Point(1, 1)}
            },

            // S-Piece
            {
                    {new Point(1, 0), new Point(2, 0), new Point(0, 1), new Point(1, 1)},
                    {new Point(0, 0), new Point(0, 1), new Point(1, 1), new Point(1, 2)},
                    {new Point(1, 0), new Point(2, 0), new Point(0, 1), new Point(1, 1)},
                    {new Point(0, 0), new Point(0, 1), new Point(1, 1), new Point(1, 2)}
            },

            // T-Piece
            {
                    {new Point(1, 0), new Point(0, 1), new Point(1, 1), new Point(2, 1)},
                    {new Point(1, 0), new Point(0, 1), new Point(1, 1), new Point(1, 2)},
                    {new Point(0, 1), new Point(1, 1), new Point(2, 1), new Point(1, 2)},
                    {new Point(1, 0), new Point(1, 1), new Point(2, 1), new Point(1, 2)}
            },

            // Z-Piece
            {
                    {new Point(0, 0), new Point(1, 0), new Point(1, 1), new Point(2, 1)},
                    {new Point(1, 0), new Point(0, 1), new Point(1, 1), new Point(0, 2)},
                    {new Point(0, 0), new Point(1, 0), new Point(1, 1), new Point(2, 1)},
                    {new Point(1, 0), new Point(0, 1), new Point(1, 1), new Point(0, 2)}
            }
    };

    private Point pieceOrigin;
    private int currentPiece;
    private int rotation;
    private ArrayList<Integer> nextPieces = new ArrayList<Integer>();

    private long score;
    private int[][] well;

    // Creates a border around the well and initializes the dropping piece
    private void init() {
        well = new int[12][24];
        for (int i = 0; i < 12; i++) {
            for (int j = 0; j < 23; j++) {
                if (i == 0 || i == 11 || j == 22) {
                    well[i][j] = 0;
                } else {
                    well[i][j] = -1;
                }
            }
        }
        newPiece();
    }

    // Put a new, random piece into the dropping position
    public void newPiece() {
        pieceOrigin = new Point(5, 2);
        rotation = 0;
        if (nextPieces.isEmpty()) {
            Collections.addAll(nextPieces, 0, 1, 2, 3, 4, 5, 6);
            Collections.shuffle(nextPieces);
        }
        currentPiece = nextPieces.get(0);
        nextPieces.remove(0);
    }

    // Collision test for the dropping piece
    private boolean collidesAt(int x, int y, int rotation) {
        for (Point p : Tetraminos[currentPiece][rotation]) {
            if (well[p.x + x][p.y + y] != -1) {
                return true;
            }
        }
        return false;
    }

    // Rotate the piece clockwise or counterclockwise
    public void rotate(int i) {
        int newRotation = (rotation + i) % 4;
        if (newRotation < 0) {
            newRotation = 3;
        }
        if (!collidesAt(pieceOrigin.x, pieceOrigin.y, newRotation)) {
            rotation = newRotation;
        }
    }

    // Move the piece left or right
    public void move(int i) {
        if (!collidesAt(pieceOrigin.x + i, pieceOrigin.y, rotation)) {
            pieceOrigin.x += i;
        }
    }

    // Drops the piece one line or fixes it to the well if it can't drop
    public void dropDown() {
        if (!collidesAt(pieceOrigin.x, pieceOrigin.y + 1, rotation)) {
            pieceOrigin.y += 1;
        } else {
            fixToWell();
        }
    }

    // Make the dropping piece part of the well, so it is available for
    // collision detection.
    public void fixToWell() {
        for (Point p : Tetraminos[currentPiece][rotation]) {
            well[pieceOrigin.x + p.x][pieceOrigin.y + p.y] = currentPiece;
        }
        clearRows();
        newPiece();
    }

    public void deleteRow(int row) {
        for (int j = row - 1; j > 0; j--) {
            for (int i = 1; i < 11; i++) {
                well[i][j + 1] = well[i][j];
            }
        }
    }

    // Clear completed rows from the field and award score according to
    // the number of simultaneously cleared rows.
    public void clearRows() {
        boolean gap;
        int numClears = 0;

        for (int j = 21; j > 0; j--) {
            gap = false;
            for (int i = 1; i < 11; i++) {
                if (well[i][j] == -1) {
                    gap = true;
                    break;
                }
            }
            if (!gap) {
                deleteRow(j);
                j += 1;
                numClears += 1;
            }
        }

        switch (numClears) {
            case 1:
                score += 100;
                break;
            case 2:
                score += 300;
                break;
            case 3:
                score += 500;
                break;
            case 4:
                score += 800;
                break;
        }
    }

    public void keyPressed(Gamepad gp) {
        if (gp.a)
            rotate(-1);
        else if (gp.b)
            rotate(+1);
        else if (gp.dpad_left)
            move(-1);
        else if (gp.dpad_right)
            move(+1);
        else if (gp.dpad_down) {
            dropDown();
            score += 1;
        }
    }

    public int state = 0;
    public boolean game = false;
    static Gamepad gp;
    ElapsedTime dropTimer = new ElapsedTime();

    public void run(Gamepad gp, Telemetry tp) {
        switch (state) {
            case 0:
                if (gp.dpad_up)
                    state++;
                break;
            case 1:
                if (!gp.dpad_up)
                    state++;
                break;
            case 2:
                if (gp.dpad_up)
                    state++;
                break;
            case 3:
                if (!gp.dpad_up)
                    state++;
                break;
            case 4:
                if (gp.dpad_down)
                    state++;
                break;
            case 5:
                if (!gp.dpad_down)
                    state++;
                break;
            case 6:
                if (gp.dpad_down)
                    state++;
                break;
            case 7:
                if (!gp.dpad_down)
                    state++;
                break;
            case 8:
                if (gp.dpad_left)
                    state++;
                break;
            case 9:
                if (!gp.dpad_left)
                    state++;
                break;
            case 10:
                if (gp.dpad_right)
                    state++;
                break;
            case 11:
                if (!gp.dpad_right)
                    state++;
                break;
            case 12:
                if (gp.dpad_left)
                    state++;
                break;
            case 13:
                if (!gp.dpad_left)
                    state++;
                break;
            case 14:
                if (gp.dpad_right)
                    state++;
                break;
            case 15:
                if (!gp.dpad_right)
                    state++;
                break;
            case 16:
                if (gp.b)
                    state++;
                break;
            case 17:
                if (!gp.b)
                    state++;
                break;
            case 18:
                if (gp.a)
                    state++;
                break;
            case 19:
                if (!gp.a)
                    state++;
                break;
            case 20:
                game = true;
//                tp.clearAll();
//                tetris.gp = gp;
//                init();
//                tp.setMsTransmissionInterval(10);
//                tp.addAction(new Runnable() {
//                    @Override
//                    public void run() {
//                        state++;
//                        keyPressed(tetris.gp);
//                        if (dropTimer.seconds() > 0.5) {
//                            dropDown();
//                            dropTimer.reset();
//                        }
//                    }
//                });
//                for (int i = 0; i < well[0].length; i++) {
//                    final int a = i;
//                    tp.addLine()
//                            .addData("t" + (a % 10), new Func<String>() {
//                                @Override
//                                public String value() {
//                                    int[] newwell = new int[well[a].length];
//
//                                    System.arraycopy(well[a], 0, newwell, 0, well[a].length);
//
//                                    String c = "";
//
//                                    for (Point p : Tetraminos[currentPiece][rotation]) {
//                                        if (pieceOrigin.y + p.y == a)
//                                            newwell[pieceOrigin.x + p.x] = currentPiece;
//                                    }
//
//                                    for (int b = 0; b < newwell.length; b++) {
//                                        switch (newwell[a][b]) {
//                                            case 0:
//                                                break;
//                                            case -1:
//                                                c += '_';
//                                                break;
//                                            default:
//                                                c += '*';
//                                        }
//                                    }
//
//                                    return c;
//                                }
//                            });
//                }
                state++;
                break;
            default:
        }
    }
}