/**
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <complex>

#include "avro/Encoder.hh"
#include "avro/Decoder.hh"
#include "avro/Specific.hh"

namespace rmf_avro {
template<typename T>
struct codec_traits<std::complex<T> > {
    static void encode(Encoder& e, const std::complex<T>& c) {
        rmf_avro::encode(e, std::real(c));
        rmf_avro::encode(e, std::imag(c));
    }

    static void decode(Decoder& d, std::complex<T>& c) {
        T re, im;
        rmf_avro::decode(d, re);
        rmf_avro::decode(d, im);
        c = std::complex<T>(re, im);
    }
};

}
int
main()
{
    std::auto_ptr<rmf_avro::OutputStream> out = rmf_avro::memoryOutputStream();
    rmf_avro::EncoderPtr e = rmf_avro::binaryEncoder();
    e->init(*out);
    std::complex<double> c1(1.0, 2.0);
    rmf_avro::encode(*e, c1);

    std::auto_ptr<rmf_avro::InputStream> in = rmf_avro::memoryInputStream(*out);
    rmf_avro::DecoderPtr d = rmf_avro::binaryDecoder();
    d->init(*in);

    std::complex<double> c2;
    rmf_avro::decode(*d, c2);
    std::cout << '(' << std::real(c2) << ", " << std::imag(c2) << ')' << std::endl;
    return 0;
}