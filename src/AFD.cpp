#include "plugin.hpp"
using simd::float_4;
using simd::int32_4;

struct AFD : Module {
	enum ParamId {
		PARAMS_LEN
	};
	enum InputId {
		IN1_INPUT,
		IN2_INPUT,
		IN3_INPUT,
		IN4_INPUT,
		INPUTS_LEN
	};
	enum OutputId {
		OUT1_OUTPUT,
		OUT2_OUTPUT,
		OUT3_OUTPUT,
		OUT4_OUTPUT,
		OUTPUTS_LEN
	};
	enum LightId {
		LIGHTS_LEN
	};

	typedef	struct {
	dsp::MinBlepGenerator<16, 32, float_4> blep;	// minBLEP function generator
	dsp::TRCFilter<float_4> sqrFilter;				// AC-coupling filter
	float_4 data[2];		// Analog shift register for input samples
	float_4 state;			// THe state of the square wave. Either -1 or +1
	float_4 delta;			// TIme constant for advancing the signal phase in terms of a sine wave
	float_4 sqr;			// The pure, unprocessed pulse output
	float_4 blp;			// minBLEP function output
	float_4 output;			// THe sum of the previous two
	float_4 mask;			// minBLEP processing mask
	int crossing;			// A bitfield, collecting 4 zero-crossing flags
	} AFDstruct;

	AFDstruct mono;			// A single 4-channel structure
	AFDstruct poly[4][4];	// A set for 4 inputs * 4 sets * 3 polyphony channels

	int32_4 channels;		// Number of polyphony channels per input

	float sampleTime;

	AFD() {
		config(PARAMS_LEN, INPUTS_LEN, OUTPUTS_LEN, LIGHTS_LEN);
		configInput(IN1_INPUT, "Input 1");
		configInput(IN2_INPUT, "Input 2");
		configInput(IN3_INPUT, "Input 3");
		configInput(IN4_INPUT, "Input 4");
		configOutput(OUT1_OUTPUT, "Output 1");
		configOutput(OUT2_OUTPUT, "Output 2");
		configOutput(OUT3_OUTPUT, "Output 3");
		configOutput(OUT4_OUTPUT, "Output 4");

		// Square wave state initialization. The absolute value is defined here.
		mono.state = -1.f;
		for(auto i=0;i<4;i++) {
			for(auto j=0;j<4;j++) {
				poly[i][j].state = -1.f;
			}
		}
	}

	inline void shift_samples_poly(AFDstruct &p, int i, int c) {
		// Shift-in samples
		p.data[1] = p.data[0];
		p.data[0] = inputs[i].getPolyVoltageSimd<float_4>(c);
	}

	void divide(AFDstruct &p) {
		// Check for any ascending zero-crossing 
		p.crossing = simd::movemask((p.data[1] < 0.f) & (p.data[0] >= 0.f));
		if(p.crossing) {
			for(int j = 0; j<4; j++) {

				// Determine the specific channel with the crossing
				if(p.crossing & (1 << j)) {
					p.mask = simd::movemaskInverse<float_4>((1 << j));

					// Frequency is divided HERE
					p.state.s[j] *= -1.f;

					// Use linear interpolation to detect the subsample time offset of the zero-crossing
					p.delta = p.mask & (p.data[0] / (p.data[1] - p.data[0]));

					// Smooth the transition
					p.blep.insertDiscontinuity(p.delta.s[j], 2.f * (p.state & p.mask));
				}
			}
		}
	}

	void prepare_output(AFDstruct &p) {
		// Mix signals
		p.blp = p.blep.process() * 5.f;
		p.sqr = p.state * 5.f;

		// Remove DC-offset
		p.sqrFilter.setCutoffFreq(20.f * sampleTime);
		p.sqrFilter.process(p.sqr + p.blp);
		p.output = p.sqrFilter.highpass() * 0.95f;
	}

	void process(const ProcessArgs &args) override {
		int i;
		sampleTime = args.sampleTime;
		// Acquire number of channels and check if there is polyphonic input anywhere
		for(i=0; i<INPUTS_LEN; i++) {
			channels.s[i] = inputs[i].getChannels();
		}
		int polytest = simd::movemask(channels > 1);
		if(polytest)
			/* The section for polyphonic processing. SIMD operations are performed on channels */
			for(i = 0; i < INPUTS_LEN; i++) {
				if((inputs[i].isConnected()) && (outputs[i].isConnected())) {
					// Unused inputs/outputs are skipped to save processing time
					for(int c = 0; c < channels.s[i]; c+=4) {
					
						shift_samples_poly(poly[i][c/4], i, c);
						divide(poly[i][c/4]);
						prepare_output(poly[i][c/4]);

						// Output
						outputs[i].setVoltageSimd(poly[i][c/4].output, c);
					}
				}
				// Must be set every time in order for polyphony to work
				outputs[i].setChannels(channels.s[i]);
			}

		else {
			/* The section for monophonic processing. SIMD operations are performed on inputs */
			/* The processing routines are duplicated and inlined for a reason */

			// Shift-in samples. No SIMD support because data addresses aren't sequential
			mono.data[1] = mono.data[0];
			for (i = 0; i < 4; i++) {
				mono.data[0].s[i] = inputs[i].getVoltage();
			}

			divide(mono);
			prepare_output(mono);

			// Output
			for (i = 0; i < 4; i++) {
				outputs[i].setVoltage(mono.output.s[i], 0);
				// Must be set every time in order for polyphony to work
				outputs[i].setChannels(1);
			}
		}
	}
};

struct AFDWidget : ModuleWidget {
	AFDWidget(AFD* module) {
		setModule(module);
		setPanel(createPanel(asset::plugin(pluginInstance, "res/AFD.svg")));

		//addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, 0)));
		addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, 0)));
		//addChild(createWidget<ScrewSilver>(Vec(RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));
		addChild(createWidget<ScrewSilver>(Vec(box.size.x - 2 * RACK_GRID_WIDTH, RACK_GRID_HEIGHT - RACK_GRID_WIDTH)));

		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(5.08, 15.24)), module, AFD::IN1_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(5.08, 43.18)), module, AFD::IN2_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(5.08, 71.12)), module, AFD::IN3_INPUT));
		addInput(createInputCentered<PJ301MPort>(mm2px(Vec(5.08, 99.06)), module, AFD::IN4_INPUT));

		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(5.08, 29.21)), module, AFD::OUT1_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(5.08, 57.15)), module, AFD::OUT2_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(5.08, 85.09)), module, AFD::OUT3_OUTPUT));
		addOutput(createOutputCentered<PJ301MPort>(mm2px(Vec(5.08, 113.03)), module, AFD::OUT4_OUTPUT));
	}
};


Model* modelAFD = createModel<AFD, AFDWidget>("AFD");