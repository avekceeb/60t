package avekceeb.joystick;

// based on https://github.com/bauerjj/Android-Simple-Bluetooth-Example

import android.Manifest;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.Intent;
import android.content.SharedPreferences;
import android.content.pm.PackageManager;
import android.os.Bundle;
import android.os.Handler;
import android.os.SystemClock;
import android.preference.PreferenceManager;
import android.support.design.widget.FloatingActionButton;
import android.support.design.widget.Snackbar;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;
import android.support.v7.app.AppCompatActivity;
import android.support.v7.widget.Toolbar;
import android.view.MotionEvent;
import android.view.View;
import android.view.Menu;
import android.view.MenuItem;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.Button;
import android.widget.ImageView;
import android.widget.ListView;
import android.widget.TextView;
import android.widget.Toast;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.io.UnsupportedEncodingException;
import java.lang.reflect.Method;
import java.util.Set;
import java.util.UUID;


public class MainActivity extends AppCompatActivity {

//    private class MyListener implements View.OnTouchListener {
//        public MyListener() {
//
//        }
//        @Override
//        public boolean onTouch(View v, MotionEvent event) {
//            return true;
//        }
//    }

    private interface WheelFunctor {
        float getSpeed(float speed, float disbalance);
    }

    private class Smooth implements WheelFunctor {
        public float getSpeed(float speed, float disbalance) {
            // upside down parabola
            float s = Math.abs(speed);
            return (-2.0f)*s*disbalance*disbalance / (amplitude*amplitude) + s;
        }
    }

    private class Sharp implements WheelFunctor {
        public float getSpeed(float speed, float disbalance) {
            float s = Math.abs(speed);
            if (disbalance < 0.0) {
                return 2.0f * (disbalance + amplitude) *
                        (disbalance + amplitude) * s / (amplitude*amplitude) - s;
            } else if (disbalance > 0.0) {
                return 2.0f * (disbalance - amplitude) *
                        (disbalance - amplitude) * s / (amplitude*amplitude) - s;
            }
            return s;
        }
    }

    private class Linear implements WheelFunctor {
        public float getSpeed(float speed, float disbalance) {
            float s = Math.abs(speed);
            if (disbalance < 0.0) {
                return 2.0f * s * disbalance / amplitude + s;
            } else if (disbalance > 0.0) {
                return -2.0f * s * disbalance / amplitude + s;
            }
            return s;
        }
    }

    private class Slow implements WheelFunctor {
        public float getSpeed(float speed, float disbalance) {
            float s = Math.abs(speed);
            if (disbalance < 0.0) {
                return s * disbalance / amplitude + s;
            } else if (disbalance > 0.0) {
                return -1 * s * disbalance / amplitude + s;
            }
            return s;
        }
    }

    private WheelFunctor getWheelFunctor(String type) {
        switch (type) {
            case "1": return new Smooth();
            case "2": return new Sharp();
            case "3": return new Linear();
            case "4": return new Slow();
            default: return new WheelFunctor() {
                @Override
                public float getSpeed(float speed, float disbalance) {
                    return speed;
                }
            };
        }
    }

    private final static int REQUEST_ENABLE_BT = 1;
    private final static int MESSAGE_READ = 2;
    private final static int CONNECTING_STATUS = 3;


    private int amplitude;
    private float delta_threshold;
    private float gap_threshold;
    private float last_x = 0;
    private float last_y = 0;
    private int commands = 0;
    private WheelFunctor wheelF;

    private TextView status;
    private ListView list;
    private TextView show;
    private BluetoothAdapter bt;
    private Set<BluetoothDevice> paired;
    private ArrayAdapter<String> bt_array;

    //private final String TAG = CrippleActivity.class.getSimpleName();
    private Handler mHandler;
    private ConnectionThread connection;
    private BluetoothSocket mBTSocket = null;
    private static final UUID BTMODULEUUID = UUID.fromString(
            "00001101-0000-1000-8000-00805F9B34FB");


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        Toolbar toolbar = (Toolbar) findViewById(R.id.toolbar);
        setSupportActionBar(toolbar);
        FloatingActionButton fab = (FloatingActionButton) findViewById(R.id.fab);
        status = (TextView) findViewById(R.id.view_status);
        show = (TextView) findViewById(R.id.view_show);
        list = (ListView) findViewById(R.id.view_devices);
        ImageView image = (ImageView)findViewById(R.id.view_target);
        fab.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                Intent modifySettings = new Intent(MainActivity.this, SettingsActivity.class);
                startActivity(modifySettings);
            }
        });
        SharedPreferences pref = PreferenceManager.getDefaultSharedPreferences(this);
        if (null == pref) {
            Toast.makeText(getApplicationContext(), "NULL!", Toast.LENGTH_SHORT).show();
            return;
        }

        // TODO: for now reading settings only once, so need to restart to apply new values
        amplitude = Integer.parseInt(pref.getString("amplitude", "75"));
        delta_threshold = amplitude * Float.parseFloat(pref.getString("sensitivity", "0.08"));
        gap_threshold = amplitude * Float.parseFloat(pref.getString("gap", "0.1"));
        String wheel_control_function = pref.getString("list", "1");
        status.setText(String.format("amp = %d func = %s", amplitude, wheel_control_function));

        wheelF = getWheelFunctor(wheel_control_function);

        bt_array = new ArrayAdapter<String>(
                this, android.R.layout.simple_list_item_1);
        // get a handle on the bluetooth radio
        bt = BluetoothAdapter.getDefaultAdapter();

        // assign model to view
        list.setAdapter(bt_array);
        list.setOnItemClickListener(mDeviceClickListener);

        // Ask for location permission if not already allowed
        if (ContextCompat.checkSelfPermission(
                this,
                Manifest.permission.ACCESS_COARSE_LOCATION) !=
                PackageManager.PERMISSION_GRANTED)
            ActivityCompat.requestPermissions(
                    this,
                    new String[] {Manifest.permission.ACCESS_COARSE_LOCATION},
                    1);


        mHandler = new Handler() {
            public void handleMessage(android.os.Message msg) {
                if (msg.what == MESSAGE_READ) {
                    String readMessage = null;
                    try {
                        readMessage = new String(
                                (byte[]) msg.obj, "UTF-8");
                    } catch (UnsupportedEncodingException e) {
                        e.printStackTrace();
                    }
                    status.setText(readMessage);
                }

                if (msg.what == CONNECTING_STATUS){
                    if (msg.arg1 == 1)
                        status.setText(
                                "Connected to Device: " + (String)(msg.obj));
                    else
                        status.setText("Connection Failed");
                }
            }
        };

        if (bt_array == null) {
            // Device does not support Bluetooth
            status.setText("Status: Bluetooth not found");
            Toast.makeText(getApplicationContext(),
                    "Bluetooth device not found!",
                    Toast.LENGTH_SHORT).show();
            return;
        }

        show.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                listPairedDevices(v);
            }
        });



        View.OnTouchListener touchHandler = new View.OnTouchListener() {
            @Override
            public boolean onTouch(View view, MotionEvent ev) {
                int action = ev.getAction();
                if (MotionEvent.ACTION_UP == action) {
                    status.setText(String.format("%5d STOP UP", ++commands));
                    if (connection != null) {
                        connection.write(
                                new byte[] {(byte)'H', (byte)'s', (byte)0, (byte)0}
                        );
                    }
                    return false;
                }
                int w = view.getWidth() / 2;
                int h = view.getHeight() / 2;
                int radius = Math.min(w, h);
                float x = ev.getX();
                float y = ev.getY();
                x -= w;
                y -= h;
                x = amplitude * x / radius;
                y = -amplitude * y / radius;
                if (delta_threshold > ((Math.abs(last_x - x) + Math.abs(last_y - y)))) {
                    // Change is too small, don't bother radio
                    return true;
                }
                last_x = x;
                last_y = y;
                if (Math.abs(y) > amplitude) {
                    status.setText("...");
                    return true;
                } else if (Math.abs(y) < gap_threshold) {
                    // zone of stop
                    status.setText(String.format("%5d STOP x=%+3.0f y=%+3.0f", ++commands, x, y));
                    if (connection != null) {
                        connection.write(
                                new byte[] {(byte)'H', (byte)'s', (byte)5, (byte)5}
                        );
                    }
                    return true;
                }
                float inner_wheel_speed = wheelF.getSpeed(y, x);
                byte left, right, dir;
                // calculate wheel speeds
                if (x > 0) {
                    // left wheel is 'outer'
                    left = (byte)(0xff & Math.round(Math.abs(y)));
                    right = (byte)(0xff & Math.round(Math.abs(inner_wheel_speed)));
                } else {
                    // right wheel is 'outer'
                    right = (byte)(0xff & Math.round(Math.abs(y)));
                    left = (byte)(0xff & Math.round(Math.abs(inner_wheel_speed)));
                }
                // calculate wheel direction
                if (y > gap_threshold) {
                    if (inner_wheel_speed > 0) {
                        dir = 'f';
                    } else {
                        if (x > 0) {
                            dir = 'r';
                        } else {
                            dir = 'l';
                        }
                    }
                } else if (y < -gap_threshold) {
                    if (inner_wheel_speed > 0) {
                        dir = 'b';
                    } else {
                        if (x > 0) {
                            dir = 'l';
                        } else {
                            dir = 'r';
                        }
                    }
                } else {
                    dir = 's';
                }
                status.setText(String.format("%5d  RUN x=%+3.0f y=%+3.0f L=%3d R=%3d %c",
                        ++commands, x, y, left, right, dir));
                byte[] command = new byte[] {
                        (byte)'H', (byte)'K', (byte)'K', (byte)'K'};
                command[1] = dir;
                command[2] = left;
                command[3] = right;
                if (connection != null) {
                    connection.write(command);
                }
                return true;
            }
        };

        image.setOnTouchListener(touchHandler);

    }

    // Why do I need these 2 below??
    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.menu_main, menu);
        return true;
    }


    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        // Handle action bar item clicks here. The action bar will
        // automatically handle clicks on the Home/Up button, so long
        // as you specify a parent activity in AndroidManifest.xml.
        int id = item.getItemId();
        //noinspection SimplifiableIfStatement
        if (id == R.id.action_settings) {
            return true;
        }
        return super.onOptionsItemSelected(item);
    }


    // Enter here after user selects "yes" or "no" to enabling radio
    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent Data){
        // Check which request we're responding to
        if (requestCode == REQUEST_ENABLE_BT) {
            // Make sure the request was successful
            if (resultCode == RESULT_OK) {
                // The user picked a contact.
                // The Intent's data Uri identifies which contact was selected.
                status.setText("Enabled");
            }
            else
                status.setText("Disabled");
        }
    }


    private void listPairedDevices(View view){
        paired = bt.getBondedDevices();
        bt_array.clear();
        if (bt.isEnabled()) {
            // put it's one to the adapter
            for (BluetoothDevice device : paired) {
                bt_array.add(device.getName() + " " + device.getAddress());
            }
            // TODO: if list is empty
//            Toast.makeText(getApplicationContext(),
//                    "Show Paired Devices", Toast.LENGTH_SHORT).show();
        }
        else
            Toast.makeText(getApplicationContext(),
                    "Bluetooth not on", Toast.LENGTH_SHORT).show();
    }


    private AdapterView.OnItemClickListener mDeviceClickListener = new AdapterView.OnItemClickListener() {
        public void onItemClick(AdapterView<?> av, View v, int arg2, long arg3) {

            if (!bt.isEnabled()) {
                Toast.makeText(getBaseContext(),
                        "Bluetooth not on", Toast.LENGTH_SHORT).show();
                return;
            }
            // TODO: crashes here. Need to do a disconnect procedure...
            if (null != mBTSocket) {
                status.setText("Already connected...");
                try {
                    mBTSocket.close();
                } catch (IOException ioe) {}
                return;
            }
            status.setText("Connecting...");
            // Get the device MAC address, which is the last 17 chars in the View
            String info = ((TextView) v).getText().toString();
            final String address = info.substring(info.length() - 17);
            final String name = info.substring(0, info.length() - 17);

            // Spawn a new thread to avoid blocking the GUI one
            new Thread()
            {
                public void run() {
                    boolean fail = false;

                    BluetoothDevice device = bt.getRemoteDevice(address);

                    try {
                        mBTSocket = createBluetoothSocket(device);
                    } catch (IOException e) {
                        fail = true;
                        Toast.makeText(getBaseContext(),
                                "Socket creation failed", Toast.LENGTH_SHORT)
                                .show();
                    }
                    // Establish the Bluetooth socket connection.
                    try {
                        mBTSocket.connect();
                    } catch (IOException e) {
                        try {
                            fail = true;
                            mBTSocket.close();
                            mHandler.obtainMessage(CONNECTING_STATUS,
                                    -1, -1)
                                    .sendToTarget();
                        } catch (IOException e2) {
                            //insert code to deal with this
                            Toast.makeText(getBaseContext(),
                                    "Socket creation failed",
                                    Toast.LENGTH_SHORT).show();
                        }
                    }
                    if (fail == false) {
                        connection = new ConnectionThread(mBTSocket);
                        connection.start();
                        mHandler.obtainMessage(CONNECTING_STATUS,
                                1, -1, name)
                                .sendToTarget();
                    }
                }
            }.start();
        }
    };


    private BluetoothSocket createBluetoothSocket(BluetoothDevice device) throws IOException {
        try {
            final Method m = device.getClass().getMethod("createInsecureRfcommSocketToServiceRecord", UUID.class);
            return (BluetoothSocket) m.invoke(device, BTMODULEUUID);
        } catch (Exception e) {
//            Log.e(TAG, "Could not create Insecure RFComm Connection",e);
            Toast.makeText(getBaseContext(), "Could not create Insecure RFComm Connection",
                    Toast.LENGTH_LONG).show();
        }
        return  device.createRfcommSocketToServiceRecord(BTMODULEUUID);
    }



    private class ConnectionThread extends Thread {

        private final BluetoothSocket mmSocket;
        private final InputStream mmInStream;
        private final OutputStream mmOutStream;

        public ConnectionThread(BluetoothSocket socket) {
            mmSocket = socket;
            InputStream tmpIn = null;
            OutputStream tmpOut = null;
            try {
                tmpIn = socket.getInputStream();
                tmpOut = socket.getOutputStream();
            } catch (IOException e) { }

            mmInStream = tmpIn;
            mmOutStream = tmpOut;
        }


        public void run() {
            byte[] buffer;// = new byte[1024];
            int bytes;
            // Keep listening to the InputStream until an exception occurs
            while (true) {
                try {
                    // Read from the InputStream
                    bytes = mmInStream.available();
                    if (bytes != 0) {
                        buffer = new byte[1024];
                        //pause and wait for rest of data.
                        // Adjust this depending on your sending speed.
                        SystemClock.sleep(100);
                        // how many bytes are ready to be read?
                        bytes = mmInStream.available();
                        // record how many bytes we actually read
                        bytes = mmInStream.read(buffer, 0, bytes);
                        mHandler.obtainMessage(MESSAGE_READ, bytes,
                                -1, buffer)
                                // Send the obtained bytes to the UI activity
                                .sendToTarget();
                    }
                } catch (IOException e) {
                    e.printStackTrace();

                    break;
                }
            }
        }

        public void write(byte[] msg) {
            try {
                mmOutStream.write(msg);
            } catch (IOException e) { }
        }

        public void close() {
            try {
                if (null != mmSocket) {
                    mmSocket.close();
                }
            } catch (IOException e) { }
        }
    }

}
