package test.lab.vision;

import android.app.AlertDialog;
import android.content.Context;
import android.content.DialogInterface;

import dji.common.error.DJIError;

/**
 * Created by dji on 2/3/16.
 */
public class DialogUtils {
    public static void showDialog(Context ctx, String str) {
        AlertDialog.Builder builder = new AlertDialog.Builder(ctx, R.style.set_dialog);
        builder.setMessage(str);
        builder.setNeutralButton(android.R.string.ok, new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialog, int which) {
                dialog.dismiss();
            }
        });
        builder.create().show();
    }

    public static void showDialog(Context ctx, int strId) {
        AlertDialog.Builder builder = new AlertDialog.Builder(ctx, R.style.set_dialog);
        builder.setMessage(strId);
        builder.setNeutralButton(android.R.string.ok, new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialog, int which) {
                dialog.dismiss();
            }
        });
        builder.create().show();
    }

    public static void showDialogBasedOnError(Context ctx, DJIError djiError) {
        if (null == djiError) {
            showDialog(ctx, R.string.success);
        } else {
            showDialog(ctx, djiError.getDescription());
        }
    }
}
