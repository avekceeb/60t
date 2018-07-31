package avekceeb.clockworkcripple;

import android.Manifest;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.pm.PackageManager;
import android.os.Handler;
import android.os.SystemClock;
import android.support.v4.content.ContextCompat;
import android.support.v7.app.AppCompatActivity;
import android.support.v4.app.ActivityCompat;
import android.os.Bundle;
import android.view.MotionEvent;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
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


public class CrippleActivity extends AppCompatActivity {

    private final static int amplitude = 255;
    private final static double pi4 = Math.PI/4;
    private TextView v_status;
    private ListView v_list;
    private TextView v_show;
    private ImageView v_image;
    private BluetoothAdapter bt;
    private Set<BluetoothDevice> paired;
    private ArrayAdapter<String> bt_array;

    //private final String TAG = CrippleActivity.class.getSimpleName();
    private Handler mHandler;
    private ConnectionThread connection;
    private BluetoothSocket mBTSocket = null;
    private static final UUID BTMODULEUUID = UUID.fromString(
            "00001101-0000-1000-8000-00805F9B34FB");

    private final static int REQUEST_ENABLE_BT = 1;
    private final static int MESSAGE_READ = 2;
    private final static int CONNECTING_STATUS = 3;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_cripple);

        v_status = (TextView)findViewById(R.id.view_status);
        v_image = (ImageView)findViewById(R.id.view_target);
        v_list = (ListView)findViewById(R.id.view_devices);
        v_show = (TextView)findViewById(R.id.view_show);

        bt_array = new ArrayAdapter<String>(
                this, android.R.layout.simple_list_item_1);
        // get a handle on the bluetooth radio
        bt = BluetoothAdapter.getDefaultAdapter();

        // assign model to view
        v_list.setAdapter(bt_array);
        v_list.setOnItemClickListener(mDeviceClickListener);

        // Ask for location permission if not already allowed
        if (ContextCompat.checkSelfPermission(
                this,
                Manifest.permission.ACCESS_COARSE_LOCATION) !=
                PackageManager.PERMISSION_GRANTED)
            ActivityCompat.requestPermissions(
                    this,
                    new String[]{Manifest.permission.ACCESS_COARSE_LOCATION},
                    1);


        mHandler = new Handler(){
            public void handleMessage(android.os.Message msg){
                if (msg.what == MESSAGE_READ) {
                    String readMessage = null;
                    try {
                        readMessage = new String(
                                (byte[]) msg.obj, "UTF-8");
                    } catch (UnsupportedEncodingException e) {
                        e.printStackTrace();
                    }
                    v_status.setText(readMessage);
                }

                if (msg.what == CONNECTING_STATUS){
                    if (msg.arg1 == 1)
                        v_status.setText(
                                "Connected to Device: " + (String)(msg.obj));
                    else
                        v_status.setText("Connection Failed");
                }
            }
        };

        if (bt_array == null) {
            // Device does not support Bluetooth
            v_status.setText("Status: Bluetooth not found");
            Toast.makeText(getApplicationContext(),
                    "Bluetooth device not found!",
                    Toast.LENGTH_SHORT).show();
        }
        else {
            v_show.setOnClickListener(new View.OnClickListener() {
                @Override
                public void onClick(View v){
                    listPairedDevices(v);
                }
            });
            
            v_image.setOnTouchListener(
                    new View.OnTouchListener() {
                        @Override
                        public boolean onTouch(View view, MotionEvent ev) {
                            byte[] command = new byte[] {
                                    (byte)'H', (byte)'K', (byte)'K', (byte)'K', (byte)'K'};
                            int action = ev.getAction();
                            if (MotionEvent.ACTION_UP == action) {
                                v_status.setText("...stop");
                                if (connection != null) {
                                    connection.write(
                                            new byte[] {(byte)'H', (byte)'s', (byte)0, (byte)0}
                                    );
                                }
                                return true;
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
                            double r = Math.sqrt(x*x + y*y);
                            if (r > amplitude) {
                                v_status.setText(String.format("...too far", x, y));
                                return true;
                            } else if (r < amplitude / 3) {
                                v_status.setText(String.format("%f %f stopped", x, y));
                                if (connection != null) {
                                    connection.write(
                                            new byte[] {(byte)'H', (byte)'s', (byte)5, (byte)5}
                                    );
                                    return true;
                                }
                            }
                            // counter-clockwise from X axis
                            double angle = Math.atan2(y, x);
                            byte dir = 's';
                            if (angle >= pi4 && angle < 3*pi4) {
                                dir = 'f';
                            } else if (Math.abs(angle) >= 3*pi4) {
                                dir = 'l';
                            } else if (angle <= -pi4 && angle > -3*pi4) {
                                dir = 'b';
                            } else if (Math.abs(angle) < pi4) {
                                dir = 'r';
                            }
                            v_status.setText(String.format("%f %f", x, y));
                            command[1] = dir;
                            command[2] = (byte)(0xff & Math.round(r));
                            command[3] = (byte)(0xff & Math.round(r));
                            if (connection != null) {
                                connection.write(command);
                            }
                            return true;
                        }
                    }
            );
        }
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
                v_status.setText("Enabled");
            }
            else
                v_status.setText("Disabled");
        }
    }

//    private void bluetoothOn(View view){
//        if (!bt.isEnabled()) {
//            Intent enableBtIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
//            startActivityForResult(enableBtIntent, REQUEST_ENABLE_BT);
//            v_status.setText("Bluetooth enabled");
//            Toast.makeText(getApplicationContext(),"Bluetooth turned on",
//                    Toast.LENGTH_SHORT).show();
//
//        }
//        else{
//            Toast.makeText(getApplicationContext(),"Bluetooth is already on", Toast.LENGTH_SHORT).show();
//        }
//    }
//
//    private void bluetoothOff(View view){
//        bt.disable(); // turn off
//        v_status.setText("Bluetooth disabled");
//        Toast.makeText(getApplicationContext(),
//                "Bluetooth turned Off", Toast.LENGTH_SHORT).show();
//    }
//
//
//    private void discover(View view){
//        // Check if the device is already discovering
//        if(bt.isDiscovering()){
//            bt.cancelDiscovery();
//            Toast.makeText(getApplicationContext(),
//                    "Discovery stopped",Toast.LENGTH_SHORT).show();
//        }
//        else {
//            if(bt.isEnabled()) {
//                bt_array.clear(); // clear items
//                bt.startDiscovery();
//                Toast.makeText(getApplicationContext(),
//                        "Discovery started", Toast.LENGTH_SHORT).show();
//                registerReceiver(blReceiver,
//                        new IntentFilter(BluetoothDevice.ACTION_FOUND));
//            }
//            else {
//                Toast.makeText(getApplicationContext(),
//                        "Bluetooth not on", Toast.LENGTH_SHORT).show();
//            }
//        }
//    }
//
//    final BroadcastReceiver blReceiver = new BroadcastReceiver() {
//        @Override
//        public void onReceive(Context context, Intent intent) {
//            String action = intent.getAction();
//            if(BluetoothDevice.ACTION_FOUND.equals(action)){
//                BluetoothDevice device = intent.getParcelableExtra(
//                        BluetoothDevice.EXTRA_DEVICE);
//                // add the name to the list
//                bt_array.add(device.getName() + " " + device.getAddress());
//                bt_array.notifyDataSetChanged();
//            }
//        }
//    };


    private void listPairedDevices(View view){
        paired = bt.getBondedDevices();
        if(bt.isEnabled()) {
            // put it's one to the adapter
            for (BluetoothDevice device : paired) {
                bt_array.add(device.getName() + " " + device.getAddress());
            }
//            Toast.makeText(getApplicationContext(),
//                    "Show Paired Devices", Toast.LENGTH_SHORT).show();
        }
        else
            Toast.makeText(getApplicationContext(),
                    "Bluetooth not on", Toast.LENGTH_SHORT).show();
    }


    private AdapterView.OnItemClickListener mDeviceClickListener = new AdapterView.OnItemClickListener() {
        public void onItemClick(AdapterView<?> av, View v, int arg2, long arg3) {

            if(!bt.isEnabled()) {
                Toast.makeText(getBaseContext(),
                        "Bluetooth not on", Toast.LENGTH_SHORT).show();
                return;
            }

            v_status.setText("Connecting...");
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

            // Get the input and output streams, using temp objects because
            // member streams are final
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
                mmSocket.close();
            } catch (IOException e) { }
        }
    }



}
