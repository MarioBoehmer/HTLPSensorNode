package devorama.htlpsensornode;

import org.apache.http.HttpResponse;
import org.apache.http.HttpStatus;
import org.apache.http.client.HttpClient;
import org.apache.http.client.methods.HttpGet;
import org.apache.http.impl.client.DefaultHttpClient;
import org.apache.http.params.BasicHttpParams;
import org.apache.http.params.HttpConnectionParams;
import org.apache.http.params.HttpParams;
import org.apache.http.util.EntityUtils;
import org.json.JSONObject;

import android.app.Activity;
import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.widget.TextView;

public class SensorOverview extends Activity {
	/** Called when the activity is first created. */
	private TextView temperature;
	private TextView humidity;
	private TextView light;
	private TextView pressure;

	@Override
	public void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		setContentView(R.layout.main);
		temperature = (TextView) findViewById(R.id.temperatureText);
		humidity = (TextView) findViewById(R.id.humidityText);
		light = (TextView) findViewById(R.id.lightText);
		pressure = (TextView) findViewById(R.id.pressureText);
		new Thread(new Runnable() {

			@Override
			public void run() {
				while (true) {
					try {
						HttpParams httpParameters = new BasicHttpParams();
						HttpConnectionParams.setConnectionTimeout(
								httpParameters, 3000);
						HttpConnectionParams.setSoTimeout(httpParameters, 3000);
						HttpClient hc = new DefaultHttpClient(httpParameters);
						HttpGet get = new HttpGet(
								"http://192.168.0.114/sensors/");
						HttpResponse rp = hc.execute(get);
						if (rp.getStatusLine().getStatusCode() == HttpStatus.SC_OK) {
							String jsontext = EntityUtils.toString(rp
									.getEntity());
							JSONObject entries = new JSONObject(jsontext);
							String temperatureInCValue = entries
									.getString("temperatureInC");
							String temperatureInFValue = entries
							.getString("temperatureInF");
							String humidityInPercentValue = entries
									.getString("humidityInPercent");
							String lightInLuxValue = entries.getString("lightInLux");
							String pressureInPaValue = entries.getString("pressureInPa");
							Message message = new Message();
							Bundle bundle = new Bundle();
							bundle.putString("temperature", temperatureInCValue + "C " + temperatureInFValue + "F");
							bundle.putString("humidity", humidityInPercentValue + "%");
							bundle.putString("light", lightInLuxValue + " lux");
							bundle.putString("pressure", pressureInPaValue + " Pa");
							message.setData(bundle);
							handler.sendMessage(message);
						}
					} catch (Exception je) {

					} finally {
						try {
							Thread.sleep(5000);
						} catch (InterruptedException e) {
							e.printStackTrace();
						}
					}
				}

			}
		}).start();
	}

	final Handler handler = new Handler() {

		@Override
		public void handleMessage(Message myMessage) {
			String temperatureMessage = myMessage.getData().getString(
					"temperature");
			String humidityMessage = myMessage.getData().getString("humidity");
			String lightMessage = myMessage.getData().getString("light");
			String pressureMessage = myMessage.getData().getString("pressure");
			temperature.setText(temperatureMessage);
			humidity.setText(humidityMessage);
			light.setText(lightMessage);
			pressure.setText(pressureMessage);
		}
	};
}